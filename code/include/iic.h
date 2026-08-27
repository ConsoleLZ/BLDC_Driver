#ifndef IIC_H
#define IIC_H
#include <stm32f10x.h>

// 初始化
void IIC_Init();

// 起始信号
uint8_t IIC_Start();

// 停止信号
void IIC_Stop();

// 使能应答信号
void IIC_Ack();

// 使能非应答信号
void IIC_Nck();

// 地址发送
uint8_t IIC_SendAddr(uint8_t addr);

// 发送一个字节
uint8_t IIC_SendByte(uint8_t byte);

// 读取一个字节
uint8_t IIC_ReadByte(uint8_t *byte);

#endif
