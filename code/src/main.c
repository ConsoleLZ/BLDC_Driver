#include <stm32f10x.h>
#include "usart.h"
#include "timer.h"
#include "utils.h"
#include "iic.h"
#include "test.h"

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
    sin_generate(sin_table, SIN_TABLE_SIZE);

    while (1)
    {
        if (sysCnt >= 10)
        {
            sysCnt = 0;
            if (speed_duty < SPEED_DUTY)
                speed_duty += SPEED_RAMP_STEP; // 软起动爬坡, ~750ms到顶
        }
        test_spwm();
    }

    return 0;
}
