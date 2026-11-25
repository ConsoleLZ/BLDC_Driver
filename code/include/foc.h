#ifndef __FOC_H_
#define __FOC_H_
#include <stm32f10x.h>

#define POLE_PAIRS 7 // 电机的极对数

#define Iq 100 // 垂直于转子的力，值越大旋转速度越快
#define Id 0 // 与转子反向重合，值越大发热越多。也就是部分电流用来发热了

typedef struct {
    int16_t Ix; // 静止的二维坐标系的x轴的值，将三相电压通过clark变化后得到
    int16_t Iy; // 静止的二维坐标系的y轴的值，将三相电压通过clark变化后得到
} FOC_COORDINATES;

#endif