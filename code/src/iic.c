#include "iic.h"
#include "utils.h"

// I2C 总线恢复: 复位后 AS5600 可能卡住 SCL/SDA 总线,
// 用 GPIO 模拟 9 个时钟脉冲 + STOP 释放总线
static void IIC_BusRecovery(void)
{
    uint8_t i;

    // 1) 将 PB10(SCL) / PB11(SDA) 切换为 GPIO 开漏输出
    GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10 |
                    GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
    GPIOB->CRH |= (GPIO_CRH_MODE10 | GPIO_CRH_MODE11);   // 50MHz 推挽(开漏)
    GPIOB->CRH |= (GPIO_CRH_CNF10_0 | GPIO_CRH_CNF11_0); // CNF=01: 开漏输出

    // 2) SCL 和 SDA 先释放为高（开漏输出写 1 = 高阻）
    GPIOB->BSRR = GPIO_Pin_10 | GPIO_Pin_11;
    Delay_us(5);

    // 3) 发送 9 个时钟脉冲 (SCL 高低切换), SDA 保持高
    for (i = 0; i < 9; i++)
    {
        GPIOB->BRR = GPIO_Pin_10;    // SCL low
        Delay_us(5);
        GPIOB->BSRR = GPIO_Pin_10;   // SCL high
        Delay_us(5);
    }

    // 4) 产生 STOP 条件: SDA low → SCL high → SDA high
    GPIOB->BRR = GPIO_Pin_11;        // SDA low
    Delay_us(5);
    GPIOB->BSRR = GPIO_Pin_10;       // SCL high
    Delay_us(5);
    GPIOB->BSRR = GPIO_Pin_11;       // SDA high (STOP)
    Delay_us(5);
}

// 初始化
void IIC_Init()
{
    /* 先开启 GPIOB 时钟, 总线恢复需要操作 PB10/PB11 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    /* 0. 复位后 I2C 总线可能被 AS5600 卡住, 先做总线恢复 */
    IIC_BusRecovery();

    /* 1. 开启 I2C 硬件时钟 */
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    /* 1.2 GPIO时钟 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    /* 2. 设置gpio的引脚的工作模式 */
    /*
        PB10->SCL
        PB11->SDA
            复用开漏输出: 既可以用于输出也可以输入. 外界要有上拉电阻.
                    用于输入的时候,最好先输出一个1,把线的控制权交给外界.

            MODE=11 CNF=11

     */
    GPIOB->CRH |= (GPIO_CRH_MODE10 | GPIO_CRH_MODE11 | GPIO_CRH_CNF10 |
                   GPIO_CRH_CNF11);

    /* 3. 设置I2C2 */
    /* 3.1 配置硬件的工作模式  I2C  */
    I2C2->CR1 = I2C_CR1_SWRST;
    I2C2->CR1 = 0;

    /* 3.2 配置给I2C设备提供的时钟的频率 36MHz*/
    I2C2->CR2 = 36;

    /* 3.3 设置标准模式=0 or 快速模式=1 */
    // 标准模式：PCLK1 / (2 * CCR) = 36MHz / 360 = 100kHz
    I2C2->CCR = 180;

    /* 3.4 时钟信号的上升沿
         100KHz的时候要求最大上升沿不超过1us(手册)。
           时钟频率是36MHz则 写入：1 /（1/36） + 1 = 37
          其实就是计算的 最大上升沿时间/时钟周期 + 1

   */
    I2C2->TRISE = 37;

    // 使能IIC
    I2C2->CR1 |= I2C_CR1_PE;
}

// // 起始信号
uint8_t IIC_Start()
{
    I2C2->CR1 |= I2C_CR1_START;

    uint16_t timeout = 0xFFFF;

    while (!(I2C2->SR1 & I2C_SR1_SB) && timeout)
    {
        timeout--;
    }

    return timeout ? 1 : 0;
}

// 停止信号
void IIC_Stop()
{
    I2C2->CR1 |= I2C_CR1_STOP;
}

// 使能应答信号
void IIC_Ack()
{
    I2C2->CR1 |= I2C_CR1_ACK;
}

// 使能非应答信号
void IIC_Nck()
{
    I2C2->CR1 &= ~I2C_CR1_ACK;
}

// 地址发送
uint8_t IIC_SendAddr(uint8_t addr)
{
    I2C2->DR = addr;

    uint16_t timeout = 0xFFFF;

    while (!(I2C2->SR1 & I2C_SR1_ADDR) && timeout)
    {
        timeout--;
    }

    if (timeout)
    {
        // 清除ADDR
        I2C2->SR2;
    }

    return timeout ? 1 : 0;
}

// 发送一个字节
uint8_t IIC_SendByte(uint8_t byte){
    // 先等待数据寄存为空
    uint16_t timeout = 0xFFFF;
    while(!(I2C2->SR1 & I2C_SR1_TXE) && timeout){
        timeout--;
    }
    
    if(timeout){
        timeout = 0xFFFF;
        I2C2->DR = byte;
        while(!(I2C2->SR1 & I2C_SR1_BTF) && timeout){
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

    while(!(I2C2->SR1 & I2C_SR1_RXNE) && timeout){
        timeout--;
    }

    if (!timeout)
    {
        return 0;
    }

    *byte = (uint8_t)I2C2->DR;
    return 1;
}
