#ifndef __TEST_H
#define __TEST_H
#include <stm32f10x.h>

#define POLE_PAIRS 7

#define U_HIGH GPIOA->ODR |= GPIO_ODR_ODR0
#define V_HIGH GPIOA->ODR |= GPIO_ODR_ODR1
#define W_HIGH GPIOA->ODR |= GPIO_ODR_ODR2

#define U_LOW GPIOA->ODR &= ~GPIO_ODR_ODR0
#define V_LOW GPIOA->ODR &= ~GPIO_ODR_ODR1
#define W_LOW GPIOA->ODR &= ~GPIO_ODR_ODR2

#define U_Enable GPIOA->ODR |= GPIO_ODR_ODR4
#define V_Enable GPIOA->ODR |= GPIO_ODR_ODR5
#define W_Enable GPIOA->ODR |= GPIO_ODR_ODR6

#define U_Disable GPIOA->ODR &= ~GPIO_ODR_ODR4
#define V_Disable GPIOA->ODR &= ~GPIO_ODR_ODR5
#define W_Disable GPIOA->ODR &= ~GPIO_ODR_ODR6

void IO_Init(void);
void test1(void);
void test2(void);

#endif