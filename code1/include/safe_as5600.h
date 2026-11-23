#ifndef SAFE_AS5600_H
#define SAFE_AS5600_H

#include <Arduino.h>
#include "SimpleFOC.h"

/* ------------------------- 编码器（抗跳变保护） -------------------------
 * AS5600 偶发 I2C 误码会返回离谱角度：对单样本变化做限幅，
 * 连续 3 次都超出最大可信步长才认定坏值，此时沿用上一次好值并累加 glitch。
 * ----------------------------------------------------------------------*/
class SafeAS5600 : public MagneticSensorI2C {
 public:
  SafeAS5600();

  float    last_good = NAN;               // 上一次可信角度
  uint32_t glitch    = 0;                 // 最近一次读数的连续误码次数

  static constexpr float MAX_STEP = 0.6f; // 单样本最大可信变化 34°

  float getSensorAngle() override;        // 带跳变保护的原始角度读取
  void  clearStats();                     // 清空误码统计
};

#endif  // SAFE_AS5600_H
