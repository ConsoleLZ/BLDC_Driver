#include <stm32f10x.h>
#include "usart.h"
#include "timer.h"
#include "iic.h"
#include "utils.h"
#include "test.h"

volatile uint16_t sysCnt;

/* 100us 定时中断 */
void TIM6_IRQHandler(void)
{
    TIM6->SR &= ~TIM_SR_UIF;
    sysCnt++;
}

/* 串口中断: 错误/空闲处理 + 接收 */
void USART1_IRQHandler(void)
{
    uint16_t sr = USART1->SR;
    uint8_t  byte;

    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE))
    {
        byte = (uint8_t)USART1->DR;
        return;
    }
    if (sr & USART_SR_IDLE)
    {
        byte = (uint8_t)USART1->DR;
        return;
    }
    if (sr & USART_SR_RXNE)
    {
        byte = (uint8_t)USART1->DR;
    }
}

int main(void)
{
    USART_Init1();
    Timer8_Init();
    Timer6_Init();
    IIC_Init();
    BLDC_SPWM_Init();

    while (1)
    {
        if (sysCnt >= 10)
        {
            sysCnt = 0;
            BLDC_SPWM_Drive();
        }
    }

    return 0;
}
