#include "foc.h"
#include <math.h>
#include "as5600.h"
#include "usart.h"
#include "main.h"

FOC_COORDINATES foc_coordinates;

/**
 * @brief 延时函数（基于 TIM4 的 sysCnt，100us 基准）
 *        在没有 TIM4 的情况下用简单忙等，供校准函数内部使用
 */
static void Calibrate_DelayMs(uint32_t ms)
{
    // 简单忙等延时：假设 72MHz 主频，循环约 1us
    // 实际使用时可替换为 DWT 或 SysTick 延时
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 8000; j++)
            __NOP();
}

/**
 * @brief 校准专用的 Park 反变换（角度强制为 0）
 *
 * 校准时我们假定电角度 = 0，这样施加的 Id 电流就是沿"假定的 d 轴"方向。
 * 电流产生的磁场会把转子吸附到这个方向。
 */
static void Park_Reverse_ZeroAngle(float id_val, float iq_val)
{
    // 角度 = 0 时，cos(0)=1, sin(0)=0
    // Ix = Id*1 - Iq*0 = Id
    // Iy = Iq*1 + Id*0 = Iq
    foc_coordinates.Ix = id_val;
    foc_coordinates.Iy = iq_val;
}

/**
 * @brief Clark 反变换，带可选的输出使能
 *        复用原有逻辑
 */
static void Clark_Reverse_Output(void)
{
    float Ialpha = foc_coordinates.Ix;
    float Ibeta = foc_coordinates.Iy;

    float Ia = Ialpha;
    float Ib = -0.5f * Ialpha - SQRT3_2 * Ibeta;
    float Ic = -0.5f * Ialpha + SQRT3_2 * Ibeta;

    // 叠加偏置，映射到 PWM 计数区间
    float ccr_a = Ia + OFFSET;
    float ccr_b = Ib + OFFSET;
    float ccr_c = Ic + OFFSET;

    // 限幅，防止溢出
    if (ccr_a < 0)
        ccr_a = 0;
    if (ccr_a > (PWM_PERIOD - 1))
        ccr_a = PWM_PERIOD - 1;
    if (ccr_b < 0)
        ccr_b = 0;
    if (ccr_b > (PWM_PERIOD - 1))
        ccr_b = PWM_PERIOD - 1;
    if (ccr_c < 0)
        ccr_c = 0;
    if (ccr_c > (PWM_PERIOD - 1))
        ccr_c = PWM_PERIOD - 1;

    // 开启三相使能
    U_Enable;
    V_Enable;
    W_Enable;

    TIM2->CCR1 = (uint16_t)ccr_a;
    TIM2->CCR2 = (uint16_t)ccr_b;
    TIM2->CCR3 = (uint16_t)ccr_c;
}

/**
 * @brief 关闭 PWM 输出（所有通道置 50% 占空比，关闭使能）
 */
static void PWM_Stop(void)
{
    // 占空比归零（或设为 OFFSET，让输出为中间值）
    TIM2->CCR1 = OFFSET;
    TIM2->CCR2 = OFFSET;
    TIM2->CCR3 = OFFSET;

    // 关闭三相使能
    U_Disable;
    V_Disable;
    W_Disable;
}

/**
 * @brief 获取补偿后的电角度
 * @return uint16_t 补偿后的电角度 (0 ~ PWM_PERIOD-1)
 */
uint16_t AS5600_ReadEAngle(void)
{
    uint16_t angle = AS5600_ReadAngle();
    uint16_t e_angle = (uint16_t)((angle * POLE_PAIRS) % PWM_PERIOD);
    uint16_t new_angle;

    if (e_angle >= E_ANGLE_OFFSET)
    {
        new_angle = e_angle - E_ANGLE_OFFSET;
    }
    else
    {
        new_angle = PWM_PERIOD - (E_ANGLE_OFFSET - e_angle);
    }

    return new_angle;
}

/**
 * @brief 电机电角度校准函数
 *
 * 工作流程：
 *   1. 关闭 PWM，让电机自由静止
 *   2. 假定电角度 = 0，施加 Id = ALIGN_ID_CURRENT 的对齐电流
 *      （Id 沿 d 轴，产生固定方向的定子磁场）
 *   3. 等待转子被磁场吸附到对齐位置
 *   4. 多次读取编码器原始角度取平均，转换为电角度
 *   5. 该电角度即为"真正电角度为 0 时编码器读到的值" = 偏移值
 *   6. 关闭电流，更新全局偏移变量
 *
 * 注意事项：
 *   - 校准过程中电机会发热，Id 值不要过大，时间不要过长
 *   - 确保机械负载较轻（或空载），转子能被顺利拉动
 *   - 首次上电或更换电机后应执行一次校准
 *
 * @return uint16_t 电角度偏移值 (范围: 0 ~ PWM_PERIOD-1)
 */
uint16_t FOC_CalibrateAngleOffset(void)
{
    // ====== 步骤 1：关闭 PWM，让电机自由静止 ======
    PWM_Stop();
    Calibrate_DelayMs(200); // 等待约 200ms 让机械惯性消退

    // ====== 步骤 2：施加对齐电流 ======
    // 假定电角度 = 0，Park 反变换直接输出 Id 和 Iq
    // 即 Ialpha = Id, Ibeta = 0（沿 alpha 轴产生磁场）
    Park_Reverse_ZeroAngle(ALIGN_ID_CURRENT, 0.0f);
    Clark_Reverse_Output();

    // ====== 步骤 3：等待转子对齐 ======
    // 给足够时间让转子被磁场吸附并稳定
    Calibrate_DelayMs(500);

    // ====== 步骤 4：多次采样取平均 ======
    const uint16_t SAMPLE_COUNT = 100;
    uint32_t sum_raw = 0;
    uint16_t raw_angle;
    uint16_t i;

    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        if (AS5600_TryReadRawAngle(&raw_angle))
        {
            sum_raw += raw_angle;
        }
        Calibrate_DelayMs(1);
    }

    uint16_t avg_raw = (uint16_t)(sum_raw / SAMPLE_COUNT);

    // ====== 步骤 5：关闭电流 ======
    PWM_Stop();
    Calibrate_DelayMs(100);

    // ====== 步骤 6：计算偏移值 ======
    // 原始角度 (12bit, 0~4095) → 机械角度 (0~3600) → 电角度
    // 电角度 = 机械角度 × 极对数
    // 该电角度即为：在我们假定"电角度=0"时，编码器实际读到的值
    uint32_t mechanical_angle = (uint32_t)avg_raw * 3600 / 4096;
    uint32_t e_angle_raw = (mechanical_angle * POLE_PAIRS) % PWM_PERIOD;
    uint16_t offset = (uint16_t)e_angle_raw;

    return offset;
}

/**
 * @brief Park 反变换（使用当前真实电角度）
 */
void Park_Reverse(void)
{
    uint16_t e_angle = AS5600_ReadEAngle();
    float rad = (float)e_angle * (2.0f * (float)M_PI) / PWM_PERIOD;

    float cos_a = cosf(rad);
    float sin_a = sinf(rad);

    foc_coordinates.Ix = Id * cos_a - foc_coordinates.Iq * sin_a;
    foc_coordinates.Iy = foc_coordinates.Iq * cos_a + Id * sin_a;
}

/**
 * @brief Clark 反变换 αβ -> abc
 *        输出 Ia Ib Ic 为带正负的相电压，叠加偏置转为定时器 CCR 计数值
 */
void Clark_Reverse(void)
{
    float Ialpha = foc_coordinates.Ix;
    float Ibeta = foc_coordinates.Iy;

    float Ia = Ialpha;
    float Ib = -0.5f * Ialpha - SQRT3_2 * Ibeta;
    float Ic = -0.5f * Ialpha + SQRT3_2 * Ibeta;

    // 叠加偏置，映射到 PWM 计数区间
    float ccr_a = Ia + OFFSET;
    float ccr_b = Ib + OFFSET;
    float ccr_c = Ic + OFFSET;

    // 限幅，防止溢出到 ARR 之外或者负数
    if (ccr_a < 0)
        ccr_a = 0;
    if (ccr_a > (PWM_PERIOD - 1))
        ccr_a = PWM_PERIOD - 1;
    if (ccr_b < 0)
        ccr_b = 0;
    if (ccr_b > (PWM_PERIOD - 1))
        ccr_b = PWM_PERIOD - 1;
    if (ccr_c < 0)
        ccr_c = 0;
    if (ccr_c > (PWM_PERIOD - 1))
        ccr_c = PWM_PERIOD - 1;

    U_Enable;
    V_Enable;
    W_Enable;

    // 输出到 TIM2 通道：PA0 CH1, PA1 CH2, PA2 CH3
    TIM2->CCR1 = (uint16_t)ccr_a;
    TIM2->CCR2 = (uint16_t)ccr_b;
    TIM2->CCR3 = (uint16_t)ccr_c;
}

// 速度PID
void Speed_PID(void)
{
    float deviation;
    static float deviation_i = 0.0f;

    deviation = target_speed - app.rpm;
    deviation_i += deviation;

    // 记得加积分限幅，防止积分饱和
    if(deviation_i > 100.0f) deviation_i = 100.0f;
    if(deviation_i < -100.0f) deviation_i = -100.0f;

    foc_coordinates.Iq = 5.0f * deviation + 0.1f * deviation_i;
}

void FOC_Work(void)
{
    Park_Reverse();
    Clark_Reverse();
}
