#include "motor_control.h"

#include <Arduino.h>
#include <Wire.h>
#include "SimpleFOC.h"
#include "config.h"
#include "tim2_driver.h"
#include "safe_as5600.h"

/* ------------------------------ 全局对象 -------------------------------- */
SafeAS5600  sensor;
Tim2Driver  driver;
BLDCMotor   motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, MOTOR_KV, PHASE_INDUCTANCE);

namespace {

/* ------------------------------ 运行状态 -------------------------------- */
enum class Mode : uint8_t { IDLE, OPEN_LOOP, CLOSED_LOOP };

Mode     mode        = Mode::IDLE;
float    target_set  = STAGE2_VELOCITY;    // 目标转速
float    target_now  = 0.0f;               // 斜坡后的实际指令转速
float    closed_vlim = CLOSED_VOLTAGE;     // 闭环电压上限
uint32_t stall_ms    = 0;                  // 堵转累计时间

/* 紧急停机：目标清零并关断输出 */
void stopOutput()
{
  target_set = 0.0f;
  target_now = 0.0f;
  motor.disable();
  mode = Mode::IDLE;
  stall_ms = 0;
}

/* 进入开环运行 */
void startOpenLoop(float v, float vlim)
{
  motor.enable();
  motor.controller = MotionControlType::velocity_openloop;
  motor.updateVoltageLimit(vlim);

  target_set = _constrain(v, -OPENLOOP_VEL_LIMIT, OPENLOOP_VEL_LIMIT);
  target_now = 0.0f;
  mode = Mode::OPEN_LOOP;
}

/* 进入闭环：标定模式下做完整对齐并打印参数；正常模式直接注入标定值，
 * initFOC() 会跳过正反扫掠，上电不产生任何对齐动作 */
void startClosedLoop(float v)
{
  sensor.clearStats();
  motor.enable();

#if ALIGN_CALIBRATE
  /* 一次性标定：完整对齐 + 串口输出 */
  Serial1.begin(115200);
  if (!motor.initFOC()) {
    motor.disable();
    startOpenLoop(STAGE2_VELOCITY, OPENLOOP_VOLTAGE);
    return;
  }
  Serial1.print("[CAL] ZERO=");
  Serial1.println(motor.zero_electric_angle, 5);
  Serial1.print("[CAL] DIR=");
  Serial1.println((int)motor.sensor_direction);
  Serial1.println("[CAL] 填入 config.h 后把 ALIGN_CALIBRATE 改为 0 重新烧录");
#else
  /* 注入标定值，跳过方向扫描与零电角标定 */
  motor.zero_electric_angle = MOTOR_ZERO_ELECTRIC_ANGLE;
  motor.sensor_direction =
      (MOTOR_SENSOR_DIRECTION >= 0) ? Direction::CW : Direction::CCW;

  if (!motor.initFOC()) {
    motor.disable();
    startOpenLoop(STAGE2_VELOCITY, OPENLOOP_VOLTAGE);
    return;
  }
#endif

  motor.controller = MotionControlType::velocity;
  motor.updateVoltageLimit(closed_vlim);
  motor.PID_velocity.reset();

  target_set = v;
  target_now = 0.0f;
  mode = Mode::CLOSED_LOOP;
}

}  // namespace

/* ----------------------------- 初始化入口 -------------------------------- */
void motorControlSetup()
{
  /* 编码器：100kHz，AS5600 只有 10k 上拉 */
  sensor.init();
  Wire.setClock(100000L);
  delay(10);

  /* 自写 TIM2 20kHz */
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit       = SUPPLY_VOLTAGE;
  if (!driver.init()) {
    while (true) delay(10);
  }

  /* 电机环参数 */
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  motor.torque_controller        = TorqueControlType::voltage;
  motor.foc_modulation           = FOCModulationType::SinePWM;
  motor.controller               = MotionControlType::velocity;
  motor.voltage_limit            = closed_vlim;
  motor.voltage_sensor_align     = ALIGN_VOLTAGE;
  motor.velocity_limit           = MAX_VELOCITY;
  motor.PID_velocity.P           = VEL_P;
  motor.PID_velocity.I           = VEL_I;
  motor.PID_velocity.D           = 0.0f;
  motor.PID_velocity.output_ramp = 2000.0f;
  motor.LPF_velocity.Tf          = VEL_TF;

  if (!motor.init()) {
    while (true) delay(10);
  }
  motor.disable();

  /* 对齐 -> 闭环，中间不做任何预检/暖机 */
  sensor.clearStats();
  startClosedLoop(STAGE2_VELOCITY);
}

/* ------------------------------ 主循环 ---------------------------------- */
void motorControlLoop()
{
  if (mode == Mode::IDLE) { delay(2); return; }

  /* 计算实际 dt，异常时用兜底值 */
  static uint32_t last_us = micros();
  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;
  if (dt <= 0.0f || dt > LOOP_DT_MAX) dt = LOOP_DT_DEFAULT;

  /* 闭环按加速度斜坡逼近目标，开环直接跟随 */
  if (mode == Mode::CLOSED_LOOP) {
    float step = VEL_RAMP * dt;
    float err  = target_set - target_now;
    if      (err >  step) target_now += step;
    else if (err < -step) target_now -= step;
    else                  target_now  = target_set;
  } else {
    target_now = target_set;
  }

  motor.loopFOC();
  motor.move(target_now);

  /* 闭环下编码器持续误码 -> 停机，绝不用错角度硬拉 */
  if (mode == Mode::CLOSED_LOOP && sensor.glitch > ENC_GLITCH_STOP) {
    stopOutput();
    return;
  }

  /* 输出有指令但电机不转 -> 判定堵转，超时停机 */
  if (mode == Mode::CLOSED_LOOP && fabs(target_now) > STALL_VEL_MIN &&
      fabs(motor.shaft_velocity) < STALL_VEL_RATIO * fabs(target_now)) {
    stall_ms += (uint32_t)(dt * 1000.0f);
    if (stall_ms > STALL_TIME_MS) stopOutput();
  } else {
    stall_ms = 0;
  }
}
