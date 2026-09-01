#include "test.h"
#include "usart.h"
#include "as5600.h"

void IO_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // GPIOA->CRL &= ~GPIO_CRL_CNF0;
    // GPIOA->CRL |= GPIO_CRL_MODE0;
    // GPIOA->CRL &= ~GPIO_CRL_CNF1;
    // GPIOA->CRL |= GPIO_CRL_MODE1;
    // GPIOA->CRL &= ~GPIO_CRL_CNF2;
    // GPIOA->CRL |= GPIO_CRL_MODE2;

    GPIOA->CRL &= ~GPIO_CRL_CNF4;
    GPIOA->CRL |= GPIO_CRL_MODE4;
    GPIOA->CRL &= ~GPIO_CRL_CNF5;
    GPIOA->CRL |= GPIO_CRL_MODE5;
    GPIOA->CRL &= ~GPIO_CRL_CNF6;
    GPIOA->CRL |= GPIO_CRL_MODE6;
}

void test1(void)
{
    static uint8_t step;

    U_Enable;
    V_Enable;
    W_Enable;
    switch (step)
    {
    case 0:
        U_HIGH;
        W_LOW;
        V_Disable;
        break;
    case 1:
        U_HIGH;
        V_LOW;
        W_Disable;
        break;
    case 2:
        W_HIGH;
        V_LOW;
        U_Disable;
        break;
    case 3:
        W_HIGH;
        U_LOW;
        V_Disable;
        break;
    case 4:
        V_HIGH;
        U_LOW;
        W_Disable;
        break;
    case 5:
        V_HIGH;
        W_LOW;
        U_Disable;
        break;
    default:
        break;
    }
    if (++step >= 6)
        step = 0;
}

void test2(void)
{
    uint16_t angle = AS5600_ReadAngle();
    uint16_t e_angle = (uint16_t)((angle * POLE_PAIRS) % 3600);
    uint8_t step = e_angle / 600;

    U_Enable;
    V_Enable;
    W_Enable;
    switch (step)
    {
    case 0:
        U_HIGH;
        W_LOW;
        V_Disable;
        break;
    case 1:
        U_HIGH;
        V_LOW;
        W_Disable;
        break;
    case 2:
        W_HIGH;
        V_LOW;
        U_Disable;
        break;
    case 3:
        W_HIGH;
        U_LOW;
        V_Disable;
        break;
    case 4:
        V_HIGH;
        U_LOW;
        W_Disable;
        break;
    case 5:
        V_HIGH;
        W_LOW;
        U_Disable;
        break;
    default:
        break;
    }
}

void test3(void)
{
    uint16_t angle = AS5600_ReadAngle();
    uint16_t e_angle = (uint16_t)((angle * POLE_PAIRS) % 3600);
    uint8_t step = e_angle / 600;

    U_Enable;
    V_Enable;
    W_Enable;
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    switch (step)
    {
    case 0:
        TIM2->CCR1 = 1800;
        W_LOW;
        V_Disable;
        break;
    case 1:
        TIM2->CCR1 = 1800;
        V_LOW;
        W_Disable;
        break;
    case 2:
        TIM2->CCR3 = 1800;
        V_LOW;
        U_Disable;
        break;
    case 3:
        TIM2->CCR3 = 1800;
        U_LOW;
        V_Disable;
        break;
    case 4:
        TIM2->CCR2 = 1800;
        U_LOW;
        W_Disable;
        break;
    case 5:
        TIM2->CCR2 = 1800;
        W_LOW;
        U_Disable;
        break;
    default:
        break;
    }
}

// 正弦表生成
void sin_generate(int16_t *sin_table, uint16_t table_size)
{
    /* 步长 d = 2π/256:  cos(d)≈0.999699,  sin(d)≈0.024531 */
    // 注意：如果单片机空间不够用，这里最好是减少步长的精度，因为这里会生成一个大数组
    // 由于浮点单片机不好算，这里转换成整型去算
    const int16_t cosd = 32758; /* 0.999699 * 32768 */
    const int16_t sind = 803;    /* 0.024531 * 32768 */

    // 初始时的x为0°
    int16_t sinx = 0;
    int16_t cosx = 32767;
    int32_t s_next, c_next;

    for (uint16_t i = 0; i < table_size; i++)
    {
        sin_table[i] = sinx;

        s_next = (int32_t)(sinx * cosd) + (int32_t)(cosx * sind);
        c_next = (int32_t)(cosx * cosd) - (int32_t)(sinx * sind);

        sinx = (int16_t)(s_next >> 15);
        cosx = (int16_t)(c_next >> 15);
    }

    /* 强制修正关键点, 消除256步累计误差: 0°/90°/180°/270° */
    sin_table[0]   = 0;
    sin_table[64]  = 32767;
    sin_table[128] = 0;
    sin_table[192] = -32767;
}

// SPWM参数
int16_t sin_table[SIN_TABLE_SIZE];

void test_spwm(void)
{
    uint16_t angle = AS5600_ReadAngle();
    uint16_t e_angle = (uint16_t)((angle * POLE_PAIRS) % 3600);

    U_Enable;
    V_Enable;
    W_Enable;
}