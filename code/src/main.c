#include <stm32f10x.h>
#include "usart.h"
#include "timer.h"
#include "utils.h"
#include "iic.h"
#include "test.h"
#include "adc.h"
#include <stdio.h>

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

int main(void)
{
    USART_Init1();
    Timer2_Init();
    Timer4_Init();
    IIC_Init();
    IO_Init();
    ADC1_Init();
    sin_generate(sin_table, SIN_TABLE_SIZE);

    while (1)
    {
        if (sysCnt >= 10)
        {
            sysCnt = 0;
            if (speed_duty < SPEED_DUTY)
                speed_duty += SPEED_RAMP_STEP; // 软起动爬坡, ~750ms到顶

            int16_t u = ADC_mV(ADC_CH_PA3) - 1650;
            int16_t v = ADC_mV(ADC_CH_PA7) - 1650;
            int16_t w = 0 - u - v;
            printf("u = %d, v = %d, w = %d\r\n", u, v, w);
            test_vector();
        }
    }

    return 0;
}
