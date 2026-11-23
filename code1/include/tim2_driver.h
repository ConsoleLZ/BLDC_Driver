#ifndef TIM2_DRIVER_H
#define TIM2_DRIVER_H

#include <Arduino.h>
#include "SimpleFOC.h"
#include "config.h"

/* --------------------------- 自写 TIM2 三相 PWM ---------------------------
 * PA0/1/2 -> TIM2_CH1/2/3，中心对齐模式，20kHz；
 * PA4/5/6 -> DRV8313 使能脚（高有效）。
 * ------------------------------------------------------------------------*/
class Tim2Driver : public BLDCDriver3PWM {
 public:
  uint16_t arr = 0;                       // 自动重装值（决定 PWM 分辨率）

  Tim2Driver();

  int  init() override;                   // 初始化 GPIO / TIM2
  void enable() override;                 // 使能 DRV8313
  void disable() override;                // 关闭 DRV8313
  void setPwm(float Ua, float Ub, float Uc) override;   // 三相占空比输出
};

#endif  // TIM2_DRIVER_H
