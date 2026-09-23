#ifndef ADC_H
#define ADC_H
#include <stm32f10x.h>

#define ADC_CH_PA3 3
#define ADC_CH_PA7 7

#define ADC_VREF_MV 3300 // 参考电压, 用万用表实测 VDDA 后改这里

void ADC1_Init(void);
uint16_t ADC_Read(uint8_t channel); // 12bit 原始值
uint16_t ADC_mV(uint8_t channel);   // 换算后的毫伏值

#endif
