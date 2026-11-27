#include <stm32f10x.h>
#include "usart.h"
#include "timer.h"
#include "utils.h"
#include "iic.h"
#include "adc.h"
#include <stdio.h>
#include "foc.h"
#include "as5600.h"

volatile uint16_t sysCnt;
volatile uint16_t _1sCnt;

// 100us中断一次
void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_UIF)
    {
        TIM4->SR &= ~TIM_SR_UIF;
        sysCnt++;
        _1sCnt++;
    }
}

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

int main(void)
{
    uint8_t speedDiv = 0;  // 测速分频：每 10 个控制周期(约10ms)取一次
    float rpm = 0.0f;

    USART_Init1();
    Timer2_Init();
    Timer4_Init();
    IIC_Init();
    IO_Init();
    ADC1_Init();

    while (1)
    {
        if (sysCnt >= 10)
        {
            sysCnt = 0;
            FOC_Work();

            // 测速函数必须 10~20ms 调用一次：调用间隔内转角超过半圈会判错方向
            if (++speedDiv >= 10)
            {
                speedDiv = 0;
                rpm = AS5600_GetSpeedRPM();
            }
        }

        if (_1sCnt >= 10000)
        {
            _1sCnt = 0;
            printf("rpm: %.2f\n", rpm);
        }
    }

    return 0;
}
