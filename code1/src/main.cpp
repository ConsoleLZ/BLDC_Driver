#include <Arduino.h>
#include <Wire.h>
#include "SimpleFOC.h"

// AS5600 I2C 磁编码器
//   库内 AS5600_I2C 配置：chip_address=0x36 / 12bit / angle_register=0x0C
//   默认复用 Wire：PB6=SCL, PB7=SDA
MagneticSensorI2C sensor = MagneticSensorI2C::AS5600();

// 2804 电机：7 极对 / 相电阻 2.55Ω / KV 220（规格书参数）
BLDCMotor motor = BLDCMotor(7, 2.55, 220);

// DRV8313 三相驱动：3PWM 模式（互补低侧由 DRV8313 内部生成）
//   PWM：PA0/PA1/PA2 → IN1/IN2/IN3
//   EN ：PA4/PA5/PA6 → EN1/EN2/EN3
BLDCDriver3PWM driver = BLDCDriver3PWM(PA0, PA1, PA2, PA4, PA5, PA6);

// 开环目标速度（rad/s），正负方向，5 rad/s ≈ 48 RPM
float target_velocity = 5.0f;

void setup()
{
  Serial1.begin(115200); // 调试串口：PA9=TX / PA10=RX
  sensor.init();         // AS5600 初始化
  analogReadResolution(12); // STM32 ADC 默认 12 位

  // —— 驱动器初始化 ——
  driver.voltage_power_supply = 12.0f; // 电机电源 12V（规格书额定）
  driver.init();
  motor.linkDriver(&driver);

  // —— 开环速度模式 ——
  // ponytail: 2V 是经验值。2804 相电阻 2.55Ω → I≈0.78A，超额定 0.5A。
  //          发热/堵转过流就降到 1.3V；带不动负载再调高，留这把刀。
  motor.voltage_limit = 2.0f;
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();
  motor.initFOC(); // 开环模式不进行对齐，但需初始化内部状态
  motor.enable();

  Serial1.println("Motor open-loop ready.");
}

void loop()
{
  // —— 开环速度控制（高频调用让运动平滑）——
  motor.move(target_velocity); // 设置目标速度
  motor.loopFOC();             // 开环下生成三相 PWM

  // —— 串口监控（100ms 一次，避免刷屏）——
  // static unsigned long last_print = 0;
  // if (millis() - last_print >= 100)
  // {
  //   last_print = millis();

  //   float deg = sensor.getSensorAngle() * 180.0f / PI; // AS5600 角度
  //   uint16_t raw_pa3 = analogRead(PA3);
  //   uint16_t raw_pa7 = analogRead(PA7);
  //   float v_pa3 = raw_pa3 * 3.3f / 4095.0f;
  //   float v_pa7 = raw_pa7 * 3.3f / 4095.0f;

  //   Serial1.print("deg=");
  //   Serial1.print(deg, 1);
  //   Serial1.print(" PA3=");
  //   Serial1.print(v_pa3, 2);
  //   Serial1.print("V PA7=");
  //   Serial1.print(v_pa7, 2);
  //   Serial1.println("V");
  // }

  delay(1); // ~1ms 控制周期
}
