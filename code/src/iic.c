#include "iic.h"
#include "utils.h"

// I2C 总线恢复: 复位后 AS5600 可能卡住 SCL/SDA 总线,
// 用 GPIO 模拟 9 个时钟脉冲 + STOP 释放总线
static void IIC_BusRecovery(void)
{
    uint8_t i;

    // 1) 将 PB6(SCL) / PB7(SDA) 切换为 GPIO 开漏输出 (CRL, pin0~7)
    GPIOB->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6 |
                    GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
    GPIOB->CRL |= (GPIO_CRL_MODE6 | GPIO_CRL_MODE7);   // 50MHz
    GPIOB->CRL |= (GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_0); // CNF=01: 开漏输出

    // 2) SCL 和 SDA 先释放为高（开漏输出写 1 = 高阻）
    GPIOB->BSRR = GPIO_Pin_6 | GPIO_Pin_7;
    Delay_us(5);

    // 3) 发送 9 个时钟脉冲 (SCL 高低切换), SDA 保持高
    for (i = 0; i < 9; i++)
    {
        GPIOB->BRR = GPIO_Pin_6;    // SCL low
        Delay_us(5);
        GPIOB->BSRR = GPIO_Pin_6;   // SCL high
        Delay_us(5);
    }

    // 4) 产生 STOP 条件: SDA low → SCL high → SDA high
    GPIOB->BRR = GPIO_Pin_7;        // SDA low
    Delay_us(5);
    GPIOB->BSRR = GPIO_Pin_6;       // SCL high
    Delay_us(5);
    GPIOB->BSRR = GPIO_Pin_7;       // SDA high (STOP)
    Delay_us(5);
}

// 初始化
void IIC_Init()
{
    /* 先开启 GPIOB 时钟, 总线恢复需要操作 PB6/PB7 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    /* 0. 复位后 I2C 总线可能被 AS5600 卡住, 先做总线恢复 */
    IIC_BusRecovery();

    /* 1. 开启 I2C1 硬件时钟 (PB6=I2C1_SCL, PB7=I2C1_SDA) */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* 2. 设置 GPIO 引脚工作模式
         PB6->SCL, PB7->SDA
         复用开漏输出: MODE=11 CNF=11 (50MHz)
         PB6/PB7 在 CRL (pin0~7)
     */
    GPIOB->CRL |= (GPIO_CRL_MODE6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF6 |
                   GPIO_CRL_CNF7);

    /* 3. 设置 I2C1 */
    /* 3.1 软件复位 */
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    /* 3.2 I2C 外设时钟频率 = PCLK1 = 36MHz */
    I2C1->CR2 = 36;

    /* 3.3 标准模式 100kHz: CCR = PCLK1 / (2 * 100k) = 36M / 200k = 180 */
    I2C1->CCR = 180;

    /* 3.4 上升沿时间: 100kHz 最大 1us → 1us / (1/36M) + 1 = 37 */
    I2C1->TRISE = 37;

    // 使能 I2C1
    I2C1->CR1 |= I2C_CR1_PE;
}

// 起始信号
uint8_t IIC_Start()
{
    I2C1->CR1 |= I2C_CR1_START;

    uint16_t timeout = 0xFFFF;

    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout)
    {
        timeout--;
    }

    return timeout ? 1 : 0;
}

// 停止信号
void IIC_Stop()
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

// 使能应答信号
void IIC_Ack()
{
    I2C1->CR1 |= I2C_CR1_ACK;
}

// 使能非应答信号
void IIC_Nck()
{
    I2C1->CR1 &= ~I2C_CR1_ACK;
}

// 地址发送
uint8_t IIC_SendAddr(uint8_t addr)
{
    I2C1->DR = addr;

    uint16_t timeout = 0xFFFF;

    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout)
    {
        timeout--;
    }

    if (timeout)
    {
        // 清除 ADDR: 读 SR1 后读 SR2
        I2C1->SR2;
    }

    return timeout ? 1 : 0;
}

// 发送一个字节
uint8_t IIC_SendByte(uint8_t byte){
    // 先等待数据寄存器为空
    uint16_t timeout = 0xFFFF;
    while(!(I2C1->SR1 & I2C_SR1_TXE) && timeout){
        timeout--;
    }
    
    if(timeout){
        timeout = 0xFFFF;
        I2C1->DR = byte;
        while(!(I2C1->SR1 & I2C_SR1_BTF) && timeout){
            timeout--;
        }

        return timeout ? 1 : 0;
    }else {
        return 0;
    }
}

// 读取一个字节
uint8_t IIC_ReadByte(uint8_t *byte){
    uint16_t timeout = 0xFFFF;

    while(!(I2C1->SR1 & I2C_SR1_RXNE) && timeout){
        timeout--;
    }

    if (!timeout)
    {
        return 0;
    }

    *byte = (uint8_t)I2C1->DR;
    return 1;
}
