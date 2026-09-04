#ifndef SIMPLE_FOC_H
#define SIMPLE_FOC_H
#include <stdint.h>

typedef enum {
    FOC_MODE_DISABLE = 0,
    FOC_MODE_SPEED   = 1,   /* 速度闭环: 机械角速度 rad/s */
    FOC_MODE_POS     = 2,   /* 位置闭环: 累计机械角度 deg */
} FOC_Mode_t;

void    SimpleFOC_Init(void);
void    SimpleFOC_Enable(void);
void    SimpleFOC_Disable(void);

/* speed: 机械角速度 rad/s. 推荐 |speed| <= 60 rad/s (~573 RPM for 7PP) */
void    SimpleFOC_SetTargetSpeed(float target_rad_per_sec);

/* pos_deg: 机械累计角度 deg, 支持多圈. 位置环相对当前位置收敛 */
void    SimpleFOC_SetTargetPosition(float target_deg);

/* 每 100us 调用一次（TIM4节拍） */
void    SimpleFOC_Run100us(void);

/* 调试观测 (只读) */
typedef struct {
    float i_alpha, i_beta;       /* Clarke (Ia,Ib采样, 旁路=0) */
    float i_d, i_q;              /* Park (D/Q轴电流, 旁路=0) */
    float v_d, v_q;              /* 电压指令 (duty半幅单位, ±PWM_PERIOD/2) */
    float theta_e_rad;           /* 当前电角度 rad */
    float speed_mech_rad_s;      /* 机械转速 rad/s (LPF后) */
    float pos_mech_deg;          /* 机械累计角度 deg (含多圈) */
    float duty_a, duty_b, duty_c;/* 三相最终占空比 0..PWM_PERIOD */
} FOC_Debug_t;
extern FOC_Debug_t foc_dbg;

#endif
