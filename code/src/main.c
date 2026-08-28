#include <stm32f10x.h>
#include "usart.h"
#include "timer.h"
#include "utils.h"
#include "iic.h"
#include "test.h"

volatile uint16_t sysCnt;

// 100us中断一次
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        // 清除中断标志
        TIM2->SR &= ~TIM_SR_UIF;
        sysCnt++;
    }
}

int main(void)
{
    USART_Init1();
    Timer2_Init();
    IIC_Init();
    IO_Init();

    while (1)
    {
        if (sysCnt >= 10)
        {
            sysCnt = 0;
            test2();
        }
    }

    return 0;
}
