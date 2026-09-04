#include "simple_foc.h"
#include "as5600.h"
#include "test.h"       /* sin_table, POLE_PAIRS, SIN_TABLE_SIZE, U_Enable/Disable macros */
#include "timer.h"      /* PWM_PERIOD */
#include <stdint.h>

/* =====================================================================
 * FOC (Field Oriented Control) - minimum viable implementation.
 * =====================================================================
 * Blocks implemented:
 *   Clarke / Park / InvPark / InvClarke transforms
 *   3 independent PID controllers (speed loop, position loop, D/Q current loops)
 *   SVPWM via zero-sequence injection (works with edge-aligned PWM)
 *   AS5600-based speed estimation with linear electrical-angle prediction
 *   Multi-turn absolute mechanical position (no wrap)
 *
 * ponytail: 电流采样 (Ia/Ib) 目前为占位宏 FOC_READ_CURRENTS，
 *   在硬件接入两相 ADC 前，电流环被旁路 —— Vq 直接由速度环输出,
 *   Vd 硬编码 0 (Id=0, 表贴 PMSM 最大转矩电流比)。替换该宏即可
 *   升级为真电流闭环，无需改动其余代码。
 * =====================================================================*/

/* -------------------- PID -------------------- */
typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    float integral;
    float prev_err;
} PID_t;

static float pid_step(PID_t *p, float target, float actual, float dt)
{
    const float err = target - actual;
    float out = p->kp * err;

    /* Anti-windup: accept integration only if output stays in range
     * OR the sign of error drives us back into range. */
    const float i_new = p->integral + p->ki * err * dt;
    const float tentative = out + i_new;
    const int inside = (tentative <= p->out_max) && (tentative >= p->out_min);
    const int pull_back =
        (tentative >  p->out_max && err < 0.0f) ||
        (tentative <  p->out_min && err > 0.0f);
    if (inside || pull_back) p->integral = i_new;

    out += p->integral;
    out += p->kd * (err - p->prev_err) / dt;
    p->prev_err = err;

    if      (out > p->out_max) out = p->out_max;
    else if (out < p->out_min) out = p->out_min;
    return out;
}

/* -------------------- Global / static state -------------------- */
FOC_Debug_t foc_dbg;

static FOC_Mode_t  foc_mode     = FOC_MODE_DISABLE;
static float       target_speed = 0.0f;   /* rad/s (mechanical) */
static float       target_pos   = 0.0f;   /* deg   (mechanical, multi-turn) */

/* ================== 调参旋钮 (有抖动/不转先按顺序改这几个) ==================
 *
 *  【第一步诊断 (必做)】把 FOC_OPENLOOP_TEST 改成 1:
 *    → 完全绕开 PID/速度反馈, 只给一个缓升的 Vq 产生旋转磁场 (等价 test_vector).
 *    → 如果这能平顺旋转: 硬件/零位/绕组顺序都OK, 问题在 PID 反馈符号 / 参数;
 *    → 如果仍然不转: 检查 OFFSET_IDX (换电机/重拆后重校), 或半桥使能 / AS5600 接线.
 *
 *  【第二步方向反转】FOC_ROTATION_REVERSE = 1 即可切换命令正方向,
 *    反馈信号保持物理真实方向 (不会导致正反馈).
 *
 *  【第三步调 PID】开环OK后关 OPENLOOP_TEST=0, 再按注释调 PID.
 * =========================================================================*/
#define FOC_OPENLOOP_TEST       0            /* 1 = 等价 test_vector 开环验证模式 */
#define FOC_ROTATION_REVERSE    0            /* 0/1 用户方向选择 */
#define SPEED_RAMP_PER_MS       0.5f         /* rad/s 每 ms 最大步长 (软启动) */
#define VQ_RAMP_PER_MS          40.0f        /* Vq 半占空比每 ms 最大步长 (转矩斜坡) */

/* 开环验证模式下的目标 Vq (半占空比单位). 从 0 缓升到该值, 保持.
 * 参考 test_vector: SPEED_DUTY=900. 先从 600 试, 平顺再加大. */
#define OPENLOOP_TARGET_VQ      600.0f
#define OPENLOOP_VQ_RAMP_PER_MS 5.0f         /* 太抖就减小. ~120ms 到 600. */

/* PID 参数 (仅 FOC_OPENLOOP_TEST = 0 生效) */
static PID_t pid_spd = { .kp =   5.0f, .ki =  10.0f, .kd = 0.0f,  .out_min = -1700, .out_max =  1700 };
static PID_t pid_pos = { .kp =   0.3f, .ki =   0.1f, .kd = 0.02f, .out_min =  -30,  .out_max =    30 };
/* 电流环占位 (启用 ADC 时生效) */
static PID_t pid_id  = { .kp =   0.2f, .ki =  20.0f, .kd = 0.0f,  .out_min = -1700, .out_max =  1700 };
static PID_t pid_iq  = { .kp =   0.2f, .ki =  20.0f, .kd = 0.0f,  .out_min = -1700, .out_max =  1700 };

/* 电角度零位校准 (和 test_vector 严格一致). 换电机/重拆装编码器后重测.
 * ponytail: 如果开环模式也不转, 先尝试 ±10 一格调整; 真 FOC (Id=0 最大转矩) 时再精调到最小电流. */
#define FOC_ELEC_OFFSET_IDX   175

#define TWO_PI            6.283185307179586f
#define SQRT3_OVER_2      0.8660254037844386f
#define INV_SQRT3         0.5773502691896257f
#define HALF_PERIOD       ((float)(PWM_PERIOD / 2))

/* 用户方向系数: 只作用于"命令 / 误差", 绝不污染反馈信号 (否则正反馈). */
#define FOC_USR_DIR   ( FOC_ROTATION_REVERSE ? -1.0f : 1.0f )

/* AS5600 / prediction state */
static uint16_t last_raw   = 0;
static int64_t  total_raw  = 0;
static float    elec_idx_f = 0.0f;        /* electrical sin-table idx, float for prediction */
static float    d_elec_per_tick = 0.0f;   /* elec_idx advance per 100us tick */
static float    spd_filt   = 0.0f;        /* mechanical speed LPF (rad/s) */

/* ----------------------------------------------------------------
 * ponytail: 电流采样占位。硬件就绪后替换为 ADC 换算值 (单位A).
 *   示例 (假设通道 PA3=Ia, PB0=Ib, 12-bit ADC, 采样电阻/运放后
 *   0~4095 对应 -I_max ~ +I_max):
 *      ia = ((int32_t)ADC1->JDR1 - 2048) * I_MAX / 2048.0f;
 *      ib = ((int32_t)ADC1->JDR2 - 2048) * I_MAX / 2048.0f;
 *   再同步调大 pid_id/pid_iq 的 Kp/Ki 即可 (和量纲匹配)。
 * ----------------------------------------------------------------*/
#define FOC_READ_CURRENTS(ia, ib)  do { (ia) = 0.0f; (ib) = 0.0f; } while (0)

/* -------------------- Public API -------------------- */
void SimpleFOC_Init(void)
{
    foc_mode = FOC_MODE_DISABLE;
    target_speed = 0.0f;
    target_pos   = 0.0f;
    pid_spd.integral = pid_spd.prev_err = 0.0f;
    pid_pos.integral = pid_pos.prev_err = 0.0f;
    pid_id.integral  = pid_id.prev_err  = 0.0f;
    pid_iq.integral  = pid_iq.prev_err  = 0.0f;

    TIM2->CCR1 = TIM2->CCR2 = TIM2->CCR3 = PWM_PERIOD / 2;

    uint16_t raw;
    if (AS5600_TryReadRawAngle(&raw)) last_raw = raw;
    total_raw  = last_raw;
    elec_idx_f = (float)((uint32_t)last_raw * POLE_PAIRS) / 16.0f + (float)FOC_ELEC_OFFSET_IDX;
    d_elec_per_tick = 0.0f;
    spd_filt = 0.0f;
}

void SimpleFOC_Enable(void)
{
    U_Enable; V_Enable; W_Enable;
    if (foc_mode == FOC_MODE_DISABLE) foc_mode = FOC_MODE_SPEED;
}

void SimpleFOC_Disable(void)
{
    U_Disable; V_Disable; W_Disable;
    TIM2->CCR1 = TIM2->CCR2 = TIM2->CCR3 = 0;
    foc_mode = FOC_MODE_DISABLE;
}

void SimpleFOC_SetTargetSpeed(float v)
{
    foc_mode = FOC_MODE_SPEED;
    target_speed = v;
}

void SimpleFOC_SetTargetPosition(float d)
{
    foc_mode = FOC_MODE_POS;
    target_pos = d;
}

/* -------------------- Scheduler entry (every 100us) -------------------- */
void SimpleFOC_Run100us(void)
{
    static uint32_t tick = 0;
    tick++;

    /* ============= 1 kHz outer loops (position + speed, every 1ms) ============= */
    if ((tick % 10U) == 0U)
    {
        /* --- 1a. 斜坡状态 (每1ms推进) --- */
        static float ramped_speed_cmd = 0.0f;
        static float ramped_vq        = 0.0f;
#if FOC_OPENLOOP_TEST
        static float openloop_vq      = 0.0f;  /* 开环缓升 Vq */
#endif
        const float usr_dir = FOC_USR_DIR;     /* 仅对命令/误差生效, 反馈不变. */

        uint16_t raw;
        if (AS5600_TryReadRawAngle(&raw))
        {
            /* signed delta (±2048 covers ±½ rev/ms = 30k RPM, 远超本电机能力) */
            int32_t delta = (int32_t)raw - (int32_t)last_raw;
            if      (delta >  2048) delta -= 4096;
            else if (delta < -2048) delta += 4096;

            last_raw   = raw;
            total_raw += delta;

            /* 瞬时机械速度: delta(raw/ms) * 2π/4.096 (rad/s).
             * 关键: 反馈值保持物理真实方向 (delta>0 = AS5600 读数增加的方向),
             * 绝不乘以方向系数, 否则破坏负反馈导致越顶越抖 (正反馈发散). */
            const float inst = (float)delta * 1.5339807878856412f;
            /* 1阶 LPF, α=0.1 → τ≈10 ms */
            spd_filt = 0.1f * inst + 0.9f * spd_filt;

            /* 刷新电角度真值 (覆盖预测积累误差).
             * elec_idx = raw*POLE_PAIRS/16 + OFFSET_IDX  (0..256 sin table 索引)
             * 与 test_vector 完全一致:  保证 FOC_ELEC_OFFSET_IDX=175 直接继承. */
            elec_idx_f = (float)((uint32_t)raw * POLE_PAIRS) / 16.0f
                       + (float)FOC_ELEC_OFFSET_IDX;

            /* 每 100us tick 电角度前进量 (线性预测, 物理方向). */
            d_elec_per_tick = (float)delta * (float)POLE_PAIRS / 160.0f;
        }

#if FOC_OPENLOOP_TEST
        /* =========================================================
         * 开环验证模式: 绕开全部 PID 闭环, Vq 固定缓升 (等价 test_vector V/f).
         * 先让硬件 / 零位 / 绕组顺序 验证通过, 再关 OPENLOOP 调 PID.
         * =========================================================*/
        {
            const float tgt = (foc_mode != FOC_MODE_DISABLE) ? OPENLOOP_TARGET_VQ : 0.0f;
            if      (openloop_vq < tgt - OPENLOOP_VQ_RAMP_PER_MS) openloop_vq += OPENLOOP_VQ_RAMP_PER_MS;
            else if (openloop_vq > tgt + OPENLOOP_VQ_RAMP_PER_MS) openloop_vq -= OPENLOOP_VQ_RAMP_PER_MS;
            else                                                   openloop_vq = tgt;
        }
        foc_dbg.v_d = 0.0f;
        foc_dbg.v_q = openloop_vq;
        pid_spd.integral = 0.0f; pid_spd.prev_err = 0.0f;
        pid_pos.integral = 0.0f; pid_pos.prev_err = 0.0f;

#else  /* 闭环模式 (PID) */

        /* --- 1b. 速度命令斜坡 (软启动) + 用户方向 --- */
        {
            /* 用户目标命令先应用方向: target_speed * usr_dir */
            const float ref_dir = (foc_mode == FOC_MODE_SPEED) ? (target_speed * usr_dir)
                                                               : ramped_speed_cmd;
            if (SPEED_RAMP_PER_MS > 0.0f) {
                if      (ramped_speed_cmd < ref_dir - SPEED_RAMP_PER_MS) ramped_speed_cmd += SPEED_RAMP_PER_MS;
                else if (ramped_speed_cmd > ref_dir + SPEED_RAMP_PER_MS) ramped_speed_cmd -= SPEED_RAMP_PER_MS;
                else                                                     ramped_speed_cmd = ref_dir;
            } else {
                ramped_speed_cmd = ref_dir;
            }
        }

        /* --- Position loop → speed target; 位置误差也应用 usr_dir 保持方向一致 --- */
        float inner_sp_target = ramped_speed_cmd;
        if (foc_mode == FOC_MODE_POS)
        {
            const float cur_deg = (float)total_raw * (360.0f / 4096.0f);
            /* 让 pos_pid 输入 = (target*usr_dir) - cur, 保证 (用户反转=1) 时
             * 目标方向跟随 usr_dir, 反馈 (cur) 仍为物理真值. */
            const float pos_ref_usrdir = target_pos * usr_dir;
            inner_sp_target = pid_step(&pid_pos, pos_ref_usrdir, cur_deg, 0.001f);
        }
        else
        {
            pid_pos.integral = 0.0f;
            pid_pos.prev_err = 0.0f;
        }

        /* --- Speed loop → Vq feedforward (current loop bypass fallback) --- */
        float vq_cmd = 0.0f;
        if (foc_mode == FOC_MODE_SPEED || foc_mode == FOC_MODE_POS)
        {
            vq_cmd = pid_step(&pid_spd, inner_sp_target, spd_filt, 0.001f);
        }
        else
        {
            pid_spd.integral = 0.0f;
            pid_spd.prev_err = 0.0f;
        }

        /* --- Vq 斜坡 (转矩限坡, 防爆冲振荡) --- */
        if (VQ_RAMP_PER_MS > 0.0f) {
            if      (ramped_vq < vq_cmd - VQ_RAMP_PER_MS) ramped_vq += VQ_RAMP_PER_MS;
            else if (ramped_vq > vq_cmd + VQ_RAMP_PER_MS) ramped_vq -= VQ_RAMP_PER_MS;
            else                                           ramped_vq = vq_cmd;
            vq_cmd = ramped_vq;
        }
        foc_dbg.v_d = 0.0f;      /* Id = 0 strategy (SPMSM) */
        foc_dbg.v_q = vq_cmd;
#endif /* FOC_OPENLOOP_TEST */

        foc_dbg.pos_mech_deg     = (float)total_raw * (360.0f / 4096.0f);
        foc_dbg.speed_mech_rad_s = spd_filt;
    }

    /* ============= 10 kHz fast FOC loop (every 100us) ============= */

    /* 1) Electrical angle prediction + wrap to [0, SIN_TABLE_SIZE) */
    elec_idx_f += d_elec_per_tick;
    float wrapped = elec_idx_f;
    /* reduce to [0, 2*SIN_TABLE_SIZE) then subtract once — cheaper than modulo */
    while (wrapped >= 2.0f * SIN_TABLE_SIZE) wrapped -= 2.0f * SIN_TABLE_SIZE;
    while (wrapped <  0.0f)                   wrapped += 2.0f * SIN_TABLE_SIZE;
    if (wrapped >= (float)SIN_TABLE_SIZE)     wrapped -= (float)SIN_TABLE_SIZE;

    const int   idx  = (int)wrapped;           /* [0, SIN_TABLE_SIZE) */
    const float frac = wrapped - (float)idx;
    const int   idx1 = (idx + 1) & (SIN_TABLE_SIZE - 1);
    const int   idxc = (idx + SIN_TABLE_SIZE / 4) & (SIN_TABLE_SIZE - 1);
    const int   idxc1= (idxc + 1) & (SIN_TABLE_SIZE - 1);

    const float s0 = (float)sin_table[idx]  / 32768.0f;
    const float s1 = (float)sin_table[idx1] / 32768.0f;
    const float sin_theta = s0 + frac * (s1 - s0);

    const float c0 = (float)sin_table[idxc]  / 32768.0f;
    const float c1 = (float)sin_table[idxc1] / 32768.0f;
    const float cos_theta = c0 + frac * (c1 - c0);

    foc_dbg.theta_e_rad = wrapped * (TWO_PI / (float)SIN_TABLE_SIZE);

    /* 2) Phase current samples (placeholder → 0) */
    float ia, ib;
    FOC_READ_CURRENTS(ia, ib);

    /* 3) Clarke transform (Ia,Ib → Iα,Iβ).  Note: Ic = -Ia-Ib implicit */
    const float i_alpha = ia;
    const float i_beta  = (ia + 2.0f * ib) * INV_SQRT3;
    foc_dbg.i_alpha = i_alpha;
    foc_dbg.i_beta  = i_beta;

    /* 4) Park transform (rotor frame → Id,Iq) */
    const float i_d =  i_alpha * cos_theta + i_beta * sin_theta;
    const float i_q = -i_alpha * sin_theta + i_beta * cos_theta;
    foc_dbg.i_d = i_d;
    foc_dbg.i_q = i_q;

    /* 5) Current-loop PI → Vd, Vq
     *   Bypass rule: 如果采样仍为 0 (ADC 未接入), 直接用外环算好的
     *   feedforward Vq, 并保持积分清零 —— 否则 0 反馈会让积分狂饱。*/
    float vd, vq;
    if (ia == 0.0f && ib == 0.0f)
    {
        vd = foc_dbg.v_d;
        vq = foc_dbg.v_q;
        pid_id.integral = 0.0f; pid_id.prev_err = 0.0f;
        pid_iq.integral = 0.0f; pid_iq.prev_err = 0.0f;
    }
    else
    {
        vd = pid_step(&pid_id, 0.0f,         i_d, 0.0001f);  /* Id_ref = 0 */
        vq = pid_step(&pid_iq, foc_dbg.v_q,  i_q, 0.0001f);  /* Iq_ref = outer loop output */
    }

    /* 6) 3-phase direct synthesis.
     *    按 test_vector 已验证的 (U-V-W 120° 正相序, +sin convention) 直接合成,
     *    绕开教科书 InvPark+InvClarke 的 U-W-V 绕组假设歧义, 100% 对齐硬件.
     *      U = Vq*sinθ  + Vd*cosθ
     *      V = Vq*sin(θ+120°) + Vd*cos(θ+120°)
     *      W = Vq*sin(θ-120°) + Vd*cos(θ-120°)
     *    用和角公式展开后只用 sinθ/cosθ (已查表), 不产生额外开销.
     * ---------------------------------------------------------------------*/
    {
        const float s = sin_theta;
        const float c = cos_theta;
        /* sin(±120°)/cos(±120°) 常数 */
        const float SIN_120 = SQRT3_OVER_2;
        const float COS_120 = -0.5f;
        /* sin(θ+120) = s*cos120 + c*sin120;   cos(θ+120) = c*cos120 - s*sin120 */
        const float s_plus  = s * COS_120 + c * SIN_120;
        const float c_plus  = c * COS_120 - s * SIN_120;
        /* sin(θ-120) = s*cos120 - c*sin120;   cos(θ-120) = c*cos120 + s*sin120 */
        const float s_minus = s * COS_120 - c * SIN_120;
        const float c_minus = c * COS_120 + s * SIN_120;

        float va = vq * s       + vd * c;
        float vb = vq * s_plus  + vd * c_plus;
        float vc = vq * s_minus + vd * c_minus;

        /* 7) SVPWM: zero-sequence injection (+15% bus utilization, symmetric pulses) */
        float vmin = va; if (vb < vmin) vmin = vb; if (vc < vmin) vmin = vc;
        float vmax = va; if (vb > vmax) vmax = vb; if (vc > vmax) vmax = vc;
        const float voff = -(vmin + vmax) * 0.5f;
        va += voff; vb += voff; vc += voff;

        /* 8) → duty cycles [0, PWM_PERIOD) + clamp */
        float da = va + HALF_PERIOD;
        float db = vb + HALF_PERIOD;
        float dc = vc + HALF_PERIOD;
        if      (da < 0)                  da = 0;
        else if (da >= (float)PWM_PERIOD) da = (float)(PWM_PERIOD - 1U);
        if      (db < 0)                  db = 0;
        else if (db >= (float)PWM_PERIOD) db = (float)(PWM_PERIOD - 1U);
        if      (dc < 0)                  dc = 0;
        else if (dc >= (float)PWM_PERIOD) dc = (float)(PWM_PERIOD - 1U);
        foc_dbg.duty_a = da; foc_dbg.duty_b = db; foc_dbg.duty_c = dc;

        if (foc_mode != FOC_MODE_DISABLE)
        {
            TIM2->CCR1 = (uint16_t)da;
            TIM2->CCR2 = (uint16_t)db;
            TIM2->CCR3 = (uint16_t)dc;
        }
    }
}
