#ifndef __FOC_H_
#define __FOC_H_
#include <stm32f10x.h>
#include "timer.h"

#define E_ANGLE_OFFSET 2948 // 电角度偏移值

#define SQRT3_2     0.8660254f   // sqrt(3)/2
#define OFFSET      (PWM_PERIOD / 2.0f)

#define POLE_PAIRS 7 // 电机的极对数

// 运行时电角度偏移值，校准后由 FOC_CalibrateAngleOffset() 更新
extern uint16_t g_e_angle_offset;

#define Iq 500.0f  // 垂直于转子的力，值越大旋转速度越快
#define Id 0.0f    // 与转子反向重合，值越大发热越多。也就是部分电流用来发热了

// 校准用的对齐电流（较大的Id值，确保转子能被拉入对齐位置）
#define ALIGN_ID_CURRENT  800.0f

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

/**
 * @brief 电机电角度校准函数
 * 
 * 原理（Alignment方法）：
 *   1. 关闭所有PWM输出，确保电机静止
 *   2. 假定电角度为0，施加 Id = ALIGN_ID_CURRENT、Iq = 0 的电流
 *   3. 定子产生固定方向的磁场，将转子吸附到该方向
 *   4. 等待转子稳定后，读取编码器角度（多次采样取平均）
 *   5. 将读取到的机械角度 × 极对数 转换为电角度
 *   6. 该值即为"电角度=0"时编码器应读到的偏移值
 *   7. 关闭电流，恢复FOC运行条件
 * 
 * @return uint16_t 电角度偏移值 (范围: 0 ~ PWM_PERIOD-1)
 */
uint16_t FOC_CalibrateAngleOffset(void);

#endif