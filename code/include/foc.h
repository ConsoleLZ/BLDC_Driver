#ifndef __FOC_H_
#define __FOC_H_
#include <stm32f10x.h>
#include "timer.h"

#define E_ANGLE_OFFSET 2040 // 电角度偏移值

#define SQRT3_2     0.8660254f   // sqrt(3)/2
#define OFFSET      (PWM_PERIOD / 2.0f)

#define POLE_PAIRS 7 // 电机的极对数

#define Iq 200.0f // 垂直于转子的力，值越大旋转速度越快
#define Id 0.0f // 与转子反向重合，值越大发热越多。也就是部分电流用来发热了

#define U_Enable GPIOA->ODR |= GPIO_ODR_ODR4
#define V_Enable GPIOA->ODR |= GPIO_ODR_ODR5
#define W_Enable GPIOA->ODR |= GPIO_ODR_ODR6

#define U_Disable GPIOA->ODR &= ~GPIO_ODR_ODR4
#define V_Disable GPIOA->ODR &= ~GPIO_ODR_ODR5
#define W_Disable GPIOA->ODR &= ~GPIO_ODR_ODR6

typedef struct {
    float Ix; // 静止的二维坐标系的x轴的值，将三相电压通过clark变化后得到
    float Iy; // 静止的二维坐标系的y轴的值，将三相电压通过clark变化后得到
} FOC_COORDINATES;

void FOC_Work(void);

#endif