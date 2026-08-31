#include <stm32f10x.h>
#include "usart.h"
#include "timer.h"
#include "utils.h"
#include "iic.h"
#include "test.h"

#define SIN_TABLE_SIZE 4096
static int16_t sin_table[SIN_TABLE_SIZE];
volatile uint16_t sysCnt;

// 100us中断一次
void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_UIF)
    {
        TIM4->SR &= ~TIM_SR_UIF; // 清除中断标志
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
            for (uint16_t i = 0; i < SIN_TABLE_SIZE; i++)
            {
                USART_Send_Byte(0xaa);
                USART_Send_Byte(sin_table[i] >> 8);
                USART_Send_Byte(sin_table[i] & 0xff);
            }
            
        }
    }

    return 0;
}
