#include <Arduino.h>

#include "motor_control.h"

/* ============================================================================
 * 2804 无刷电机 + DRV8313 + AS5600 上电自动运行
 *
 * 代码结构：
 *   include/config.h        全部宏定义与可调参数
 *   include/tim2_driver.h   TIM2 三相 PWM 驱动
 *   include/safe_as5600.h   抗跳变编码器
 *   include/motor_control.h 电机控制状态机
 * ==========================================================================*/

void setup()
{
  motorControlSetup();
}

void loop()
{
  motorControlLoop();
}
