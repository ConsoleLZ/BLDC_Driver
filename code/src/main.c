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
        if (sysCnt >= 30000)
        {
            sysCnt = 0;
            // test_spwm();
            for (uint8_t i = 0; i < (SIN_TABLE_SIZE - 1); i++)
            {
                printf("index:%d, value:%d\n", i, sin_table[i]);
            }
        }
    }

    return 0;
}
