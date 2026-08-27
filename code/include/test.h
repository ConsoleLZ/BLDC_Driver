#ifndef __TEST_H
#define __TEST_H
#include <stm32f10x.h>
#include <stdint.h>

/* ============ 电机配置 ============ */
#define POLE_PAIRS              7

/*
 * 安装相关参数。
 * 若电机只抖不转，先将 SENSOR_DIRECTION 改为 -1；仍不对时交换任意两根
 * 电机相线。POLE_PAIRS 必须与电机实际极对数一致。
 */
#define SENSOR_DIRECTION         1

/* TIM8：72 MHz / 3600 = 20 kHz，避开可闻频段。 */
#define PWM_PERIOD               3600U
#define PWM_CENTER               (PWM_PERIOD / 2U)

/* 无电流环时必须保守地给电压。可在确认相序正确后逐步提高 RUN_AMPLITUDE。 */
#define ALIGN_AMPLITUDE          360U    /* 10%，用于上电定向 */
#define RUN_AMPLITUDE            1800U    /* 20%，正常运行上限 */
#define ALIGN_TIME_MS            300U
#define SENSOR_BOOT_TIME_MS      20U
#define RAMP_TIME_MS             500U
#define FAULT_RETRY_MS           1000U
#define MAX_SENSOR_ERRORS        3U

/* 相对于转子磁场的 q 轴角。符号由 SENSOR_DIRECTION/相序共同决定。 */
#define TORQUE_ADVANCE           1024U   /* 90 electrical degrees, 4096 counts/rev */

/* ============ 旧方波测试宏 (保留兼容) ============ */
#define U_HIGH GPIOC->ODR |= GPIO_ODR_ODR6;
#define V_HIGH GPIOC->ODR |= GPIO_ODR_ODR7;
#define W_HIGH GPIOC->ODR |= GPIO_ODR_ODR8;

#define U_LOW GPIOC->ODR &= ~GPIO_ODR_ODR6;
#define V_LOW GPIOC->ODR &= ~GPIO_ODR_ODR7;
#define W_LOW GPIOC->ODR &= ~GPIO_ODR_ODR8;

/* ============ 接口函数 ============ */
void motorDriverInit(void);
void testMotorDriver(void);
void testMotorDriver1(void);
void testMotorDriverPwm(void);
void BLDC_SPWM_Init(void);

/**
 * @brief SPWM 合成磁场驱动
 *        根据 AS5600 转子位置计算电气角, 输出三相互差120°的正弦PWM
 *        在 1ms 定时循环里调用即可
 */
void BLDC_SPWM_Drive(void);

#endif
