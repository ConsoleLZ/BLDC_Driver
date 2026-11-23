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
bool     aligned     = false;              // FOC 对齐是否已完成
bool     i2c_ok      = true;               // 上电编码器总线自检结果
float    target_set  = START_VELOCITY;     // 目标转速
float    target_now  = 0.0f;               // 斜坡后的实际指令转速
float    closed_vlim = CLOSED_VOLTAGE;     // 闭环电压上限
uint32_t stall_ms    = 0;                  // 堵转累计时间
float    stage_ms    = 0.0f;               // 暖机段累计时间
uint8_t  stage       = 0;                  // 0=暖机段 1=稳态段

/* 上电自检：统计编码器坏样本数，判断 I2C 是否可信 */
bool encoderSane()
{
  uint32_t bad = 0;
  for (int i = 0; i < ENC_CHECK_SAMPLES; i++) {
    sensor.getSensorAngle();
    if (sensor.currWireError) bad++;
    delayMicroseconds(ENC_CHECK_DELAY_US);
  }
  sensor.clearStats();
  return bad <= ENC_CHECK_BAD_MAX;
}

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

/* 开机开环预检：确认功率级和电机接线正常，再停下 */
void openLoopPretest(float v, uint32_t ms)
{
  motor.enable();
  motor.controller = MotionControlType::velocity_openloop;
  motor.updateVoltageLimit(OPENLOOP_VOLTAGE);
  mode = Mode::OPEN_LOOP;

  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    motor.loopFOC();
    motor.move(v);
  }
  motor.disable();
  delay(300);
}

/* 进入闭环：编码器不可信则退回开环，否则先对齐再切速度环 */
void startClosedLoop(float v)
{
  if (!i2c_ok) {                       // 编码器不可信就只跑开环
    startOpenLoop(STAGE2_VELOCITY, OPENLOOP_VOLTAGE);
    return;
  }

  if (!aligned) {
    int ok = 0;
    for (int i = 0; i < ALIGN_RETRY && !ok; i++) {
      sensor.clearStats();
      motor.enable();
      ok = motor.initFOC();
      if (!ok) { motor.disable(); delay(500); }
      if (sensor.glitch > 20) break;
    }
    if (!ok) {
      startOpenLoop(STAGE2_VELOCITY, OPENLOOP_VOLTAGE);
      return;
    }
    aligned = true;
  } else {
    motor.enable();
  }

  motor.controller = MotionControlType::velocity;
  motor.updateVoltageLimit(closed_vlim);
  motor.PID_velocity.reset();

  stage      = 0;
  stage_ms   = 0.0f;
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

  /* 静默自检：编码器总线是否可用（约 50ms，无打印） */
  i2c_ok = encoderSane();

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

  /* 开环验证 -> 对齐 -> 闭环 */
  delay(300);
  openLoopPretest(PRETEST_VELOCITY, PRETEST_MS);
  sensor.clearStats();
  startClosedLoop(START_VELOCITY);
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

  /* 暖机 2 秒后升到稳态转速 */
  if (mode == Mode::CLOSED_LOOP && stage == 0) {
    stage_ms += dt * 1000.0f;
    if (stage_ms > WARMUP_MS) {
      stage = 1;
      target_set = STAGE2_VELOCITY;
    }
  }

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
