#ifndef USART_H
#define USART_H
#include <stm32f10x.h>
#include <stdio.h>

// 串口初始化
void USART_Init1();

// 发送一个字节
void USART_Send_Byte(uint8_t byte);

// 发送字符串
void USART_Send_String(uint8_t *str, uint8_t len);

// 接收一个字节
uint8_t USART_Receive_Byte();

// 接收字符串
void USART_Receive_String(uint8_t buffer[], uint8_t *len);

#endif