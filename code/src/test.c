#include "test.h"
#include "usart.h"

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