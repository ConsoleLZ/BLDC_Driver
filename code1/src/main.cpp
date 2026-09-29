#include <Arduino.h>
#include <Wire.h>
#include "SimpleFOC.h"

// AS5600 I2C 磁编码器实例
//   库内 AS5600_I2C 配置：chip_address=0x36 / 12bit / angle_register=0x0C
//   默认复用 Wire：PB6=SCL, PB7=SDA（STM32F103 Arduino core 默认引脚）
MagneticSensorI2C sensor = MagneticSensorI2C::AS5600();

// 电机定义
BLDCMotor motor = BLDCMotor(7, 2.55, 220);

// 驱动器
// BLDCDriver3PWM driver = BLDCDriver3PWM(
//     PA1, PA1, PA2,    // PWM 引脚
//     PA4, PA5, PA6,      // 使能引脚
//     12.0           // 电源电压
// );

void setup()
{
  Serial1.begin(115200); // 调试串口：PA9=TX / PA10=RX
  sensor.init();         // AS5600 初始化
  analogReadResolution(12); // STM32 ADC 默认 12 位，显式设置避免歧义
}

void loop()
{
  // —— AS5600 角度（I2C） ——
  float rad = sensor.getSensorAngle(); // 读取角度（弧度，0~2π）
  float deg = rad * 180.0f / PI;       // 转换为角度（0~360°）

  // —— PA3 / PA7 电压（ADC） ——
  //   STM32F103 ADC 默认参考电压 Vref = 3.3V，12 位分辨率（0~4095）
  uint16_t raw_pa3 = analogRead(PA3);
  uint16_t raw_pa7 = analogRead(PA7);
  float v_pa3 = raw_pa3 * 3.3f / 4095.0f; // 转换为电压
  float v_pa7 = raw_pa7 * 3.3f / 4095.0f;

  Serial1.print("PA3=");
  Serial1.print(v_pa3, 3);
  Serial1.print("V  PA7=");
  Serial1.print(v_pa7, 3);
  Serial1.println("V");

  delay(500);
}
