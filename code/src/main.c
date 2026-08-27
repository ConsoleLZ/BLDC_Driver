#include <stm32f10x.h>
#include "usart.h"
#include "timer.h"
#include "utils.h"
#include "iic.h"
#include "as5600.h"

volatile uint16_t sysCnt, testCnt;

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
    
    while (1)
    {
        if (sysCnt >= 10)
        {
            sysCnt = 0;
            if(++testCnt>=500){
                testCnt = 0;

                u16 angle = AS5600_ReadRawAngle();
                USART_Send_Byte(0x66);
                USART_Send_Byte(angle >> 8);
                USART_Send_Byte(angle & 0xff);
            }
        }
    }

    return 0;
}
