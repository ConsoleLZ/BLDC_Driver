#ifndef AS5600_H
#define AS5600_H
#include <stdint.h>

#define AS5600_ADDR         0x36
#define AS5600_RAW_ANGLE_HIGH  0x0C

// 读取原始角度 (12-bit, 0-4095)
uint16_t AS5600_ReadRawAngle(void);

/*
 * 读取原始角度并报告通信状态。电机控制必须使用此接口：
 * 0 也是一个合法角度，不能把通信失败误当成角度 0。
 */
uint8_t AS5600_TryReadRawAngle(uint16_t *raw_angle);

// 读取角度并转换为度数 (0-3599, 表示0.0-359.9度)
uint16_t AS5600_ReadAngle(void);

/**
 * @brief 获取当前转速
 * @return float 转速，单位 RPM，正数表示原始角度递增方向
 * 这个算法的前提是：相邻两次采样之间，机械转角绝对值一定小于半圈（2048 count）。
 */
float AS5600_GetSpeedRPM(void);

#endif
