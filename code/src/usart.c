#include "usart.h"

void USART_Init1()
{
    // 开启时钟
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // 配置GPIO工作模式
    GPIOA->CRH |= GPIO_CRH_CNF9_1;
    GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
    GPIOA->CRH |= GPIO_CRH_MODE9;

    GPIOA->CRH &= ~GPIO_CRH_CNF10_1;
    GPIOA->CRH |= GPIO_CRH_CNF10_0;
    GPIOA->CRH &= ~GPIO_CRH_MODE10;

    // 设置波特率为115200
    USART1->BRR = 0x271;

    // 串口使能
    USART1->CR1 |= USART_CR1_UE;
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_RE;

    // 开启中断
    USART1->CR1 |= USART_CR1_RXNEIE; /* 接收非空中断 */
    USART1->CR1 |= USART_CR1_IDLEIE; /* 空闲中断 */

    NVIC_SetPriorityGrouping(3);
    /* 4.2 设置优先级 */
    NVIC_SetPriority(USART1_IRQn, 2);
    /* 4.3 使能串口1的中断 */
    NVIC_EnableIRQ(USART1_IRQn);
}

void USART_Send_Byte(uint8_t byte)
{
    // 等待数据发送完成
    while (!(USART1->SR & USART_SR_TXE))
        ;

    USART1->DR = byte;
}

void USART_Send_String(uint8_t *str, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        USART_Send_Byte(str[i]);
    }

    if (len)
    {
        USART_Send_Byte('\n');
    }
}