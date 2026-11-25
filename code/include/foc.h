#ifndef __FOC_H_
#define __FOC_H_
#include <stm32f10x.h>

#define Iq 100 // 垂直于转子的力，值越大旋转速度越快
#define Id 0 // 与转子反向重合，值越大发热越多。也就是部分电流用来发热了

typedef struct {
    uint16_t x; // 静止的二维坐标系的x轴，将三相电压通过clark变化后得到
    uint16_t y; // 静止的二维坐标系的y轴，将三相电压通过clark变化后得到
} FOC_COORDINATES;

#endif