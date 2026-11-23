#include "safe_as5600.h"

SafeAS5600::SafeAS5600()
    : MagneticSensorI2C(MagneticSensorI2C::AS5600()) {}

/* 读取原始角度，遇到跳变最多重试 3 次；全失败则沿用上次可信值 */
float SafeAS5600::getSensorAngle()
{
  for (int i = 0; i < 3; i++) {
    float a = MagneticSensorI2C::getSensorAngle();

    if (isnan(last_good)) { last_good = a; return a; }

    /* 角度差归一到 [-PI, PI] */
    float d = a - last_good;
    if (d >  (float)PI) d -= 2.0f * (float)PI;
    if (d < -(float)PI) d += 2.0f * (float)PI;

    if (fabs(d) <= MAX_STEP) { last_good = a; glitch = 0; return a; }
  }
  glitch++;
  return last_good;
}

void SafeAS5600::clearStats()
{
  glitch = 0;
}
