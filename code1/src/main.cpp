#include <Arduino.h>
#include <Wire.h>
#include "SimpleFOC.h"

/* ============================================================================
 * 2804 无刷电机(12N14P, 7 对极) + DRV8313 + AS5600 — 上电自动运行 v8（静默版）
 *
 * 已删掉全部串口打印 / 自检 / 噪声测试 / 串口命令解析，只保留控制逻辑。
 *
 * 【改转速看这里】
 *   STAGE2_VELOCITY  —— 稳态转速（默认 3.0 rad/s ≈ 28.6 rpm）★主要改这个
 *   START_VELOCITY   —— 上电头 2 秒的暖机转速（想一步到位就改成和上面一样）
 *   VEL_RAMP         —— 加速度 rad/s²，越大起步越猛
 *   CLOSED_VOLTAGE   —— 闭环最高输出电压，想更快就得抬高（默认 2.5V）
 *   MAX_VELOCITY     —— 目标速度硬上限
 *
 *   换算：rpm = rad/s × 9.5493 ；rad/s = rpm × 0.10472
 *
 * 硬件事实（实测确认，勿改）：
 *   SystemCoreClock = 64MHz（STM32duino 通用 variant 用 HSI/2×16，不用外部晶振）
 *   PA0/1/2 = TIM2_CH1/2/3 三相 PWM，中心对齐 20kHz
 *   PA4/5/6 = DRV8313 EN1/2/3（高有效）
 *   PB6/PB7 = I2C1 -> AS5600 (0x36)，100kHz，12bit
 * ==========================================================================*/

/* ----------------------------- 电机/电源参数 ----------------------------- */
#define SUPPLY_VOLTAGE      12.0f
#define POLE_PAIRS          7
#define PHASE_RESISTANCE    2.55f
#define MOTOR_KV            220.0f
#define PHASE_INDUCTANCE    0.00086f

/* ------------------------------- 引脚定义 -------------------------------- */
#define PIN_PWM_U           PA0
#define PIN_PWM_V           PA1
#define PIN_PWM_W           PA2
#define PIN_EN_U            PA4
#define PIN_EN_V            PA5
#define PIN_EN_W            PA6

/* ------------------------------- 运行参数 -------------------------------- */
#define PWM_FREQ_HZ         20000UL
#define ALIGN_VOLTAGE       2.5f
#define OPENLOOP_VOLTAGE    2.0f
/**
 * 转速升高后反电动势（Ke = 0.0434 V/(rad/s)）会吃掉电压：稳态需要 `U ≈ Ke·ω + I·R`。2.5 V 幅度下理论空载极限约 `2.5/0.0434 ≈ 57 rad/s`（≈550 rpm），
 * 但带载和摩擦会先撞墙。想明显提速就把 `CLOSED_VOLTAGE` 一起抬（12 V 供电下建议不超过 4.0 V，否则电流和发热上来，2.55 Ω / 0.5 A 额定的线圈扛不住）。
 */
#define CLOSED_VOLTAGE      2.5f

#define START_VELOCITY      1.0f       // 暖机段转速 rad/s
#define STAGE2_VELOCITY     20.94f       // ★稳态转速 rad/s（改这里）
#define WARMUP_MS           2000.0f    // 暖机段持续时间
/**
 * 改速度时别忘它得 ≥ 你的目标。另外闭环启动时速度环会按 `STAGE2_VELOCITY` 直接算出 q 轴电压，如果设太大而 `CLOSED_VOLTAGE` 不够，就会卡在限幅上失速
 */
#define MAX_VELOCITY        40.0f
#define VEL_RAMP            2.0f

#define STALL_TIME_MS       3000
#define ALIGN_RETRY         3
#define PRETEST_MS          800
#define PRETEST_VELOCITY    1.5f

/* 速度环：J=3.7e-6kg·m²，带宽≈4600·P=230rad/s(37Hz)，PI零点40rad/s */
#define VEL_P               0.05f
#define VEL_I               2.0f
#define VEL_TF              0.002f

/* --------------------------- 自写 TIM2 三相 PWM --------------------------- */
class Tim2Driver : public BLDCDriver3PWM {
 public:
  uint16_t arr = 0;

  Tim2Driver() : BLDCDriver3PWM(PIN_PWM_U, PIN_PWM_V, PIN_PWM_W,
                               PIN_EN_U, PIN_EN_V, PIN_EN_W) {}

  int init() override {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    AFIO->MAPR &= ~AFIO_MAPR_TIM2_REMAP;

    GPIOA->CRL &= ~0x00000FFFUL;
    GPIOA->CRL |=  0x00000BBBUL;                 // PA0/1/2 AF_PP 50MHz

    GPIOA->CRL &= ~0x0FFF0000UL;
    GPIOA->CRL |=  0x03330000UL;                 // PA4/5/6 推挽 50MHz
    GPIOA->BRR   = (1UL << 4) | (1UL << 5) | (1UL << 6);

    arr = (uint16_t)((SystemCoreClock / (2UL * PWM_FREQ_HZ)) - 1UL);   // 64M -> 1599

    TIM2->PSC  = 0;
    TIM2->ARR  = arr;
    TIM2->CR1  = TIM_CR1_ARPE | TIM_CR1_CMS_0;
    TIM2->CCMR1 = (6UL << 4) | TIM_CCMR1_OC1PE | (6UL << 12) | TIM_CCMR1_OC2PE;
    TIM2->CCMR2 = (6UL << 4) | TIM_CCMR2_OC3PE;
    TIM2->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM2->CCR1  = TIM2->CCR2 = TIM2->CCR3 = arr / 2;
    TIM2->EGR   = TIM_EGR_UG;
    TIM2->CR1  |= TIM_CR1_CEN;

    initialized = true;
    return 1;
  }

  void enable() override {
    GPIOA->BSRR = (1UL << 4) | (1UL << 5) | (1UL << 6);
    setPwm(0, 0, 0);
  }

  void disable() override {
    setPwm(0, 0, 0);
    GPIOA->BRR = (1UL << 4) | (1UL << 5) | (1UL << 6);
  }

  void setPwm(float Ua, float Ub, float Uc) override {
    Ua = _constrain(Ua, 0.0f, voltage_limit);
    Ub = _constrain(Ub, 0.0f, voltage_limit);
    Uc = _constrain(Uc, 0.0f, voltage_limit);

    dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    TIM2->CCR1 = (uint16_t)(dc_a * (arr + 1));
    TIM2->CCR2 = (uint16_t)(dc_b * (arr + 1));
    TIM2->CCR3 = (uint16_t)(dc_c * (arr + 1));
  }
};

/* ------------------------- 编码器（抗跳变保护） ------------------------- */
class SafeAS5600 : public MagneticSensorI2C {
 public:
  SafeAS5600() : MagneticSensorI2C(MagneticSensorI2C::AS5600()) {}

  float    last_good = NAN;
  uint32_t glitch    = 0;

  static constexpr float MAX_STEP = 0.6f;      // 单样本最大可信变化 34°

  float getSensorAngle() override {
    for (int i = 0; i < 3; i++) {
      float a = MagneticSensorI2C::getSensorAngle();

      if (isnan(last_good)) { last_good = a; return a; }

      float d = a - last_good;
      if (d >  (float)PI) d -= 2.0f * (float)PI;
      if (d < -(float)PI) d += 2.0f * (float)PI;

      if (fabs(d) <= MAX_STEP) { last_good = a; glitch = 0; return a; }
    }
    glitch++;
    return last_good;
  }

  void clearStats() { glitch = 0; }
};

SafeAS5600  sensor;
Tim2Driver  driver;
BLDCMotor  motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, MOTOR_KV, PHASE_INDUCTANCE);

/* ------------------------------- 运行状态 -------------------------------- */
enum class Mode : uint8_t { IDLE, OPEN_LOOP, CLOSED_LOOP };

Mode  mode        = Mode::IDLE;
bool  aligned     = false;
bool  i2c_ok      = true;
float target_set  = START_VELOCITY;
float target_now  = 0.0f;
float closed_vlim = CLOSED_VOLTAGE;
uint32_t stall_ms = 0;
float    stage_ms = 0.0f;
uint8_t  stage    = 0;

/* ------------------------------- 内部函数 -------------------------------- */
static bool encoderSane()
{
  uint32_t bad = 0;
  for (int i = 0; i < 120; i++) {
    sensor.getSensorAngle();
    if (sensor.currWireError) bad++;
    delayMicroseconds(400);
  }
  sensor.clearStats();
  return bad <= 20;
}

static void stopOutput()
{
  target_set = 0.0f;
  target_now = 0.0f;
  motor.disable();
  mode = Mode::IDLE;
  stall_ms = 0;
}

static void startOpenLoop(float v, float vlim)
{
  motor.enable();
  motor.controller = MotionControlType::velocity_openloop;
  motor.updateVoltageLimit(vlim);

  target_set = _constrain(v, -10.0f, 10.0f);
  target_now = 0.0f;
  mode = Mode::OPEN_LOOP;
}

static void openLoopPretest(float v, uint32_t ms)
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

static void startClosedLoop(float v)
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

/* ------------------------------- setup/loop ------------------------------ */
void setup()
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

void loop()
{
  if (mode == Mode::IDLE) { delay(2); return; }

  static uint32_t last_us = micros();
  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.002f;

  /* 暖机 2 秒后升到稳态转速 */
  if (mode == Mode::CLOSED_LOOP && stage == 0) {
    stage_ms += dt * 1000.0f;
    if (stage_ms > WARMUP_MS) {
      stage = 1;
      target_set = STAGE2_VELOCITY;
    }
  }

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
  if (mode == Mode::CLOSED_LOOP && sensor.glitch > 40) {
    stopOutput();
    return;
  }

  if (mode == Mode::CLOSED_LOOP && fabs(target_now) > 1.0f &&
      fabs(motor.shaft_velocity) < 0.15f * fabs(target_now)) {
    stall_ms += (uint32_t)(dt * 1000.0f);
    if (stall_ms > STALL_TIME_MS) stopOutput();
  } else {
    stall_ms = 0;
  }
}
