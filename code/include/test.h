#ifndef __TEST_H
#define __TEST_H
#include <stm32f10x.h>

#define POLE_PAIRS 7
#define SIN_TABLE_SIZE 256

#define SPEED_DUTY 1800 // 速度

// 三相索引偏移 (120°/360° * 256 = 85.3, 取 85 / 171)
#define IDX_OFF_U       0
#define IDX_OFF_V       85
#define IDX_OFF_W       171

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

extern int16_t sin_table[];

void IO_Init(void);
void test1(void);
void test2(void);
void test3(void);
void sin_generate(int16_t *sin_table, uint16_t table_size);
void test_spwm(void);

#endif