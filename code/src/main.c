#include <stm32f10x.h>
#include "usart.h"
#include "timer.h"
#include "utils.h"
#include "iic.h"
#include "adc.h"
#include <stdio.h>
#include "foc.h"

volatile uint16_t sysCnt;

// 100us中断一次
void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_UIF)
    {
        TIM4->SR &= ~TIM_SR_UIF;
        sysCnt++;
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
        }

        FOC_Work();
    }

    return 0;
}
