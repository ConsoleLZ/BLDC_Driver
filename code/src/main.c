#include <stm32f10x.h>
#include "usart.h"
#include "timer.h"
#include "utils.h"

volatile uint16_t sysCnt;

/* 100us 定时中断 (TIM4, F103C8T6 中容量确认存在的通用定时器) */
void TIM4_IRQHandler(void)
{
    TIM4->SR &= ~TIM_SR_UIF;
    sysCnt++;
}

int main(void)
{
    USART_Init1();
    
    while (1)
    {
        if (sysCnt >= 10)
        {
            sysCnt = 0;
            USART_Send_Byte(0x55);
        }
    }

    return 0;
}
