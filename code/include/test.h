#ifndef __TEST_H
#define __TEST_H
#include <stm32f10x.h>

#define POLE_PAIRS 7
#define SIN_TABLE_SIZE 256

#define E_ANGLE_OFFSET 2040 // 电角度偏移值, 设置idx为0，打印的(uint16_t)((angle * POLE_PAIRS) % PWM_PERIOD)的值
#define MOVE_OFFSET 64 // 固定90度的偏移值，只有受力方向与转子90度垂直，才能拉着电机旋转，且没有额外的分立做其它功

#define SPEED_DUTY 600 // 目标速度
#define SPEED_RAMP_STEP 2 // 软起动每ms增量, 调大=加速起, 调小=更软
extern uint16_t speed_duty; // 当前速度, 软起动用

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
void test_vector(void);

#endif