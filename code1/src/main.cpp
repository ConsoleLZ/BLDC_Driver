#include <Arduino.h>
#include <Wire.h>
#include "SimpleFOC.h" // SimpleFOC 已 vendored，含 MagneticSensorI2C（AS5600 I2C 驱动现成）

// AS5600 I2C 磁编码器实例
//   库内 AS5600_I2C 配置：chip_address=0x36 / 12bit / angle_register=0x0C
//   默认复用 Wire：PB6=SCL, PB7=SDA（STM32F103 Arduino core 默认引脚）
MagneticSensorI2C sensor = MagneticSensorI2C::AS5600();

void setup()
{
  Serial1.begin(115200); // 调试串口：PA9=TX / PA10=RX
  sensor.init();         // AS5600 初始化
}

void loop()
{
  float rad = sensor.getSensorAngle(); // 读取角度（弧度，0~2π）
  float deg = rad * 180.0f / PI;       // 转换为角度（0~360°）

  Serial1.print("raw_rad=");
  Serial1.print(rad, 4);
  Serial1.print("  deg=");
  Serial1.println(deg, 2);

  delay(500); // 约 20Hz 刷新
}
