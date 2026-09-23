#include "adc.h"
#include "utils.h"

void ADC1_Init(void)
{
    // ADC1 + GPIOA 时钟
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;

    // PCLK2=72MHz, ADC 上限 14MHz, 6 分频 -> 12MHz
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

    // PA3 / PA7 模拟输入: CNF=00, MODE=00
    GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
    GPIOA->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7);

    // 单次转换, 不用扫描/DMA/中断, 序列长度 1
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;
    ADC1->SQR1 = 0;

    // 采样时间 239.5 周期: 分压电阻阻值大时保证充满
    ADC1->SMPR2 |= ADC_SMPR2_SMP3 | ADC_SMPR2_SMP7;

    // 上电 -> 等 tSTAB -> 复位校准 -> 自校准
    ADC1->CR2 |= ADC_CR2_ADON;
    Delay_us(2);

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL)
        ;

    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL)
        ;
}

uint16_t ADC_Read(uint8_t channel)
{
    ADC1->SQR3 = channel;
    ADC1->SR &= ~ADC_SR_EOC;

    ADC1->CR2 |= ADC_CR2_ADON; // 已上电, 再写 1 触发单次转换
    while (!(ADC1->SR & ADC_SR_EOC))
        ;

    return (uint16_t)ADC1->DR;
}

uint16_t ADC_mV(uint8_t channel)
{
    return (uint16_t)((uint32_t)ADC_Read(channel) * ADC_VREF_MV / 4095);
}
