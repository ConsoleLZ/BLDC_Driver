#include <Arduino.h>
#include <Wire.h>
#include "SimpleFOC.h"

// AS5600 I2C磁编码器
MagneticSensorI2C sensor = MagneticSensorI2C::AS5600();

// 2804电机：7极对 / 相电阻2.55Ω / KV220
BLDCMotor motor = BLDCMotor(7, 2.55, 220);

// DRV8313 3PWM驱动
// PWM：PA0 PA1 PA2 → IN1 IN2 IN3
// EN：PA4 PA5 PA6 → EN1 EN2 EN3
BLDCDriver3PWM driver = BLDCDriver3PWM(PA0, PA1, PA2, PA4, PA5, PA6);

float target_velocity = 5.0f; // rad/s，目标转速

void setup() {
  Serial1.begin(115200);
  sensor.init();
  analogReadResolution(12);

  // 驱动器配置
  driver.voltage_power_supply = 12.0f;
  driver.pwm_frequency = 35000;
  driver.init();
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);  // 绑定编码器【闭环核心】

  // FOC参数
  motor.voltage_limit = 2.0f;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // 速度环PI调参（初始预设值，后续微调）
  motor.PID_velocity.P = 0.25f;
  motor.PID_velocity.I = 5.0f;
  motor.PID_velocity.D = 0.001f;
  motor.PID_velocity.output_ramp = 100.0f; // 加速度限幅，抑制冲击噪声
  motor.LPF_velocity = 0.05f; // 速度低通滤波【修正大小写】

  motor.controller = MotionControlType::velocity; // 闭环速度模式

  motor.init();
  motor.initFOC(); // 闭环必须！上电相位对齐，电机会小幅转动
  motor.enable();

  Serial1.println("Closed-loop velocity ready.");
}

void loop() {
  motor.move(target_velocity);
  motor.loopFOC();

  // 可选串口打印，用于调参，取消注释即可
  /*
  static unsigned long last_print = 0;
  if(millis()-last_print >=100){
    last_print = millis();
    Serial1.print("vel:");
    Serial1.print(motor.shaft_velocity,2);
    Serial1.print(" rad/s, angle:");
    Serial1.println(motor.shaft_angle,2);
  }
  */
}
