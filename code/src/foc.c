#include "foc.h"
#include <math.h>
#include "as5600.h"
#include "usart.h"

FOC_COORDINATES foc_coordinates;

// 获取电角度，已经计算偏移值
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
 * Park反变换
 */

void Park_Reverse(void)
{
    uint16_t e_angle = AS5600_ReadEAngle();
    float rad = (float)e_angle * (2.0f * (float)M_PI) / PWM_PERIOD;

    float cos_a = cosf(rad);
    float sin_a = sinf(rad);

    foc_coordinates.Ix = Id * cos_a - Iq * sin_a;
    foc_coordinates.Iy = Iq * cos_a + Id * sin_a;
}

/**
 * Clark反变换 αβ -> abc
 * 输出 Ia Ib Ic 为带正负的相电压（FOC里这里实际是电压量，不是电流）
 * 然后叠加偏置转为定时器CCR计数值
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

void FOC_Work(void)
{
    Park_Reverse();
    Clark_Reverse();

    // uint16_t offset = AS5600_ReadEAngle();
    // USART_Send_Byte(0xFC);
    // USART_Send_Byte(offset >> 8);
    // USART_Send_Byte(offset & 0xff);
}