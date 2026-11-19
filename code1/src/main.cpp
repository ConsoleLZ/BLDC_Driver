#include <Arduino.h>
#include <Wire.h>
#include "SimpleFOC.h"

/* ============================================================================
 * 2804 无刷电机(12N14P, 7 对极) + DRV8313 + AS5600 —— 上电自动运行 v4
 *
 * v4 关键改动（针对 v3 两个实测故障）：
 *   1. 【PWM】不再用 SimpleFOC 的 STM32 定时器路径（实测 TIM2 时钟都没开、
 *      ARR=0，引脚上其实跑的是内核 analogWrite 的 1kHz）。
 *      改成自己写寄存器：PA0/1/2 = TIM2_CH1/2/3，PSC=0、ARR=1799、
 *      中心对齐 → 严格 20kHz，并把寄存器读回来打印做闭环验证。
 *   2. 【编码器】v3 实测 200ms 内 i2c_err 从 542 涨到 1123，角度是坏的。
 *      改 100kHz、上电先做 200 次读测试、读失败自动重读一次，
 *      并统计 wire 错误率；误码率过高时只允许低压开环，不进 FOC。
 *   3. 上电流程：PWM 寄存器自检 -> 1s 三相 50% 直通(可示波器量 PA0)
 *      -> I2C 自检 -> 开环 2s -> 自动对齐 -> 闭环
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
#define PWM_FREQ_HZ         20000
#define PWM_ARR             ((72000000UL / (2UL * PWM_FREQ_HZ)) - 1UL)   // 1799
#define ALIGN_VOLTAGE       2.5f
#define OPENLOOP_VOLTAGE    1.5f
#define CLOSED_VOLTAGE      3.0f
#define MAX_VELOCITY        40.0f
#define START_VELOCITY      2.0f
#define VEL_RAMP            4.0f
#define STALL_TIME_MS       3000
#define ALIGN_RETRY         3
#define OPENLOOP_TEST_MS    2000
#define I2C_SELFTEST_N      200
#define I2C_BAD_LIMIT       40          // 自检里允许的最大失败次数

/* --------------------------- 自写 TIM2 三相 PWM --------------------------- */
/* 完全绕开 SimpleFOC 的 STM32 定时器管理，寄存器直接配置，频率可控可验证 */
class Tim2Driver : public BLDCDriver3PWM {
 public:
  Tim2Driver() : BLDCDriver3PWM(PIN_PWM_U, PIN_PWM_V, PIN_PWM_W,
                               PIN_EN_U, PIN_EN_V, PIN_EN_W) {}

  int init() override {
    /* 时钟 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* TIM2 用默认引脚映射（PA0/1/2 不需要重映射） */
    AFIO->MAPR &= ~AFIO_MAPR_TIM2_REMAP;

    /* PA0/PA1/PA2：复用推挽 50MHz (MODE=11, CNF=10 -> 每个 nibble 0xB) */
    GPIOA->CRL &= ~0x00000FFFUL;
    GPIOA->CRL |=  0x00000BBBUL;
    /* PA4/PA5/PA6：通用推挽 50MHz (MODE=11, CNF=00 -> 0x3)，先输出低（失能） */
    GPIOA->CRL &= ~0x0FFF0000UL;
    GPIOA->CRL |=  0x03330000UL;
    GPIOA->BRR   = (1UL << 4) | (1UL << 5) | (1UL << 6);

    /* 时基：72MHz / 1 / (1799+1) / 2 = 20kHz，中心对齐模式1 */
    TIM2->PSC  = 0;
    TIM2->ARR  = PWM_ARR;
    TIM2->CR1  = TIM_CR1_ARPE | TIM_CR1_CMS_0;
    /* CH1/CH2/CH3：PWM1 模式(110) + 预装载 */
    TIM2->CCMR1 = (6UL << 4) | TIM_CCMR1_OC1PE | (6UL << 12) | TIM_CCMR1_OC2PE;
    TIM2->CCMR2 = (6UL << 4) | TIM_CCMR2_OC3PE;
    TIM2->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM2->CCR1  = TIM2->CCR2 = TIM2->CCR3 = PWM_ARR / 2;
    TIM2->EGR   = TIM_EGR_UG;              // 立刻装载 PSC/ARR
    TIM2->CR1  |= TIM_CR1_CEN;

    initialized = true;
    return 1;
  }

  void enable() override {
    GPIOA->BSRR = (1UL << 4) | (1UL << 5) | (1UL << 6);
    setPwm(0, 0, 0);                       // 三相同为 0 -> 低边导通，制动
  }

  void disable() override {
    setPwm(0, 0, 0);
    GPIOA->BRR = (1UL << 4) | (1UL << 5) | (1UL << 6);   // EN 拉低 -> 输出 Hi-Z
  }

  void setPwm(float Ua, float Ub, float Uc) override {
    Ua = _constrain(Ua, 0.0f, voltage_limit);
    Ub = _constrain(Ub, 0.0f, voltage_limit);
    Uc = _constrain(Uc, 0.0f, voltage_limit);

    dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    TIM2->CCR1 = (uint16_t)(dc_a * (PWM_ARR + 1UL));
    TIM2->CCR2 = (uint16_t)(dc_b * (PWM_ARR + 1UL));
    TIM2->CCR3 = (uint16_t)(dc_c * (PWM_ARR + 1UL));
  }
};

/* ------------------------------- 编码器 -------------------------------- */
/* AS5600 读错时库会返回 0，角度瞬间跳 180°，FOC 会直接抖成筛子 */
class SafeAS5600 : public MagneticSensorI2C {
 public:
  SafeAS5600() : MagneticSensorI2C(MagneticSensorI2C::AS5600()) {}

  float    last_good = NAN;
  uint32_t glitch    = 0;   // 连续异常次数（判为不可信）
  uint32_t wire_err  = 0;   // Wire 层错误次数

  float getSensorAngle() override {
    float a = MagneticSensorI2C::getSensorAngle();
    if (currWireError) wire_err++;

    if (isnan(last_good)) { last_good = a; return a; }

    // 读失败 或 单次跳变 >90°（物理不可能）→ 立刻重读一次
    if (currWireError || fabs(a - last_good) > 1.6f) {
      float b = MagneticSensorI2C::getSensorAngle();
      if (!currWireError && fabs(b - last_good) <= 1.6f) {
        last_good = b;
        return b;
      }
      glitch++;
      return last_good;                  // 沿用上次可信角度
    }
    last_good = a;
    glitch = 0;
    return a;
  }
  void clearStats() { glitch = 0; wire_err = 0; }
};

SafeAS5600  sensor;
Tim2Driver  driver;
BLDCMotor  motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, MOTOR_KV, PHASE_INDUCTANCE);

/* ------------------------------- 运行状态 -------------------------------- */
enum class Mode : uint8_t { IDLE, OPEN_LOOP, CLOSED_LOOP };

Mode  mode        = Mode::IDLE;
bool  aligned     = false;
float target_set  = START_VELOCITY;
float target_now  = 0.0f;
float closed_vlim = CLOSED_VOLTAGE;
uint32_t stall_ms = 0;
bool  i2c_ok      = true;

char    cmd_buf[32];
uint8_t cmd_len = 0;

/* ------------------------------- 自检打印 -------------------------------- */
static void reportPwmRegs()
{
  float f = (float)72000000UL / (float)(TIM2->PSC + 1) / (float)(TIM2->ARR + 1) / 2.0f;
  Serial1.print("[PWM] PSC="); Serial1.print(TIM2->PSC);
  Serial1.print(" ARR=");      Serial1.print(TIM2->ARR);
  Serial1.print(" CR1=0x");    Serial1.print(TIM2->CR1, HEX);
  Serial1.print(" CCMR1=0x");  Serial1.print(TIM2->CCMR1, HEX);
  Serial1.print(" CCER=0x");   Serial1.print(TIM2->CCER, HEX);
  Serial1.print(" CCR=");      Serial1.print(TIM2->CCR1);
  Serial1.print("/");          Serial1.print(TIM2->CCR2);
  Serial1.print("/");          Serial1.print(TIM2->CCR3);
  Serial1.print(" => f=");     Serial1.print(f, 1);
  Serial1.println(" Hz");
}

/* 三相 50% 直通：相间 0V 差，不出力，可放心用示波器量 PA0 */
static void pwmCarrierTest()
{
  Serial1.println("[PWM] 1s 50% test on PA0/PA1/PA2 (no torque)");
  driver.enable();
  driver.setPwm(SUPPLY_VOLTAGE * 0.5f, SUPPLY_VOLTAGE * 0.5f, SUPPLY_VOLTAGE * 0.5f);
  delay(1000);
  driver.disable();
  delay(100);
  reportPwmRegs();
}

/* 上电读 200 次角度，先分清"接线/上拉问题"还是"PWM 干扰问题" */
static void i2cSelfTest()
{
  uint32_t bad = 0;
  float a_min = 1e9f, a_max = -1e9f;
  for (int i = 0; i < I2C_SELFTEST_N; i++) {
    float a = sensor.getSensorAngle();
    if (sensor.currWireError) bad++;
    if (!isnan(a)) { if (a < a_min) a_min = a; if (a > a_max) a_max = a; }
    delayMicroseconds(500);
  }
  i2c_ok = (bad <= I2C_BAD_LIMIT);

  Serial1.print("[I2C] 200 reads, wire_err=");
  Serial1.print(bad);
  Serial1.print("  span=");
  Serial1.print((a_max - a_min) * 180.0f / PI, 2);
  Serial1.print("deg  -> ");
  Serial1.println(i2c_ok ? "OK" : "UNSTABLE");
  sensor.clearStats();
}

/* ------------------------------- 辅助函数 -------------------------------- */
static void printStatus()
{
  const char* m = (mode == Mode::IDLE) ? "IDLE"
                : (mode == Mode::OPEN_LOOP) ? "OPENLOOP" : "CLOSED";
  Serial1.print("[");
  Serial1.print(m);
  Serial1.print("] ang=");
  Serial1.print(motor.shaft_angle * 180.0f / PI, 1);
  Serial1.print("deg vel=");
  Serial1.print(motor.shaft_velocity, 2);
  Serial1.print(" tgt=");
  Serial1.print(target_now, 2);
  Serial1.print(" i2c_err=");
  Serial1.print(sensor.glitch);
  Serial1.print("/");
  Serial1.println(sensor.wire_err);
}

static void stopOutput(const char* why)
{
  target_set = 0.0f;
  target_now = 0.0f;
  motor.disable();
  mode = Mode::IDLE;
  stall_ms = 0;
  Serial1.print("[STOP] ");
  Serial1.println(why);
}

static void startOpenLoop(float v, float vlim)
{
  motor.enable();
  motor.controller = MotionControlType::velocity_openloop;
  motor.updateVoltageLimit(vlim);

  target_set = _constrain(v, -10.0f, 10.0f);
  target_now = 0.0f;
  mode = Mode::OPEN_LOOP;
  Serial1.print("[OPENLOOP] vlim=");
  Serial1.print(vlim, 2);
  Serial1.print("V target=");
  Serial1.println(target_set, 2);
}

static void openLoopPretest(float v, uint32_t ms)
{
  Serial1.print("[OPEN] pretest ");
  Serial1.print(v, 1);
  Serial1.print(" rad/s ");
  Serial1.print(ms);
  Serial1.println(" ms");
  motor.enable();
  motor.controller = MotionControlType::velocity_openloop;
  motor.updateVoltageLimit(OPENLOOP_VOLTAGE);

  uint32_t t0 = millis(), t1 = t0;
  while (millis() - t0 < ms) {
    motor.loopFOC();
    motor.move(v);
    if (millis() - t1 >= 200) { t1 = millis(); printStatus(); }
  }
  motor.disable();
  delay(300);
}

static void startClosedLoop(float v)
{
  if (!i2c_ok) {
    Serial1.println("[FOC] skipped: encoder unstable, open-loop only");
    startOpenLoop(1.5f, 1.0f);
    return;
  }

  if (!aligned) {
    int ok = 0;
    for (int i = 0; i < ALIGN_RETRY && !ok; i++) {
      Serial1.print("[ALIGN] try "); Serial1.print(i + 1); Serial1.println("/3");
      sensor.clearStats();
      motor.enable();
      ok = motor.initFOC();
      if (!ok) { motor.disable(); delay(500); }
      if (sensor.glitch > 20) {
        Serial1.println("[ALIGN] encoder errors during align -> abort");
        motor.disable();
        i2c_ok = false;
        startOpenLoop(1.5f, 1.0f);
        return;
      }
    }
    if (!ok) {
      Serial1.println("[ALIGN] FAILED -> open-loop fallback");
      startOpenLoop(1.5f, 1.0f);
      return;
    }
    aligned = true;
    Serial1.print("[ALIGN] OK dir=");
    Serial1.print((int)motor.sensor_direction);
    Serial1.print(" zero_el=");
    Serial1.print(motor.zero_electric_angle, 4);
    Serial1.print(" pp=");
    Serial1.println(motor.pp_check_result ? "OK" : "FAIL");
  } else {
    motor.enable();
  }

  motor.controller = MotionControlType::velocity;
  motor.updateVoltageLimit(closed_vlim);
  motor.PID_velocity.reset();
  target_set = _constrain(v, -MAX_VELOCITY, MAX_VELOCITY);
  target_now = 0.0f;
  mode = Mode::CLOSED_LOOP;
  Serial1.print("[CLOSED] vlim="); Serial1.print(closed_vlim, 2);
  Serial1.print("V target=");      Serial1.println(target_set, 2);
}

static void execCommand(char* line)
{
  char  c   = line[0];
  char* arg = line + 1;
  while (*arg == ' ') arg++;
  float v = atof(arg);

  switch (c) {
    case 'o': startOpenLoop(arg[0] ? v : 2.0f, OPENLOOP_VOLTAGE); break;
    case 'c': startClosedLoop(arg[0] ? v : START_VELOCITY); break;
    case 't':
      if (!arg[0]) { printStatus(); break; }
      if (mode == Mode::IDLE) { Serial1.println("ERR: not running"); break; }
      target_set = _constrain(v, -MAX_VELOCITY, MAX_VELOCITY);
      break;
    case 'v':
      if (!arg[0]) { Serial1.print("vlim="); Serial1.println(closed_vlim, 2); break; }
      closed_vlim = _constrain(v, 0.5f, SUPPLY_VOLTAGE);
      if (mode != Mode::IDLE)
        motor.updateVoltageLimit((mode == Mode::OPEN_LOOP) ? OPENLOOP_VOLTAGE : closed_vlim);
      break;
    case 'p': motor.PID_velocity.P = v; break;
    case 'i': motor.PID_velocity.I = v; break;
    case 'f': motor.LPF_velocity.Tf = _constrain(v, 0.0f, 0.2f); break;
    case 's': target_set = 0.0f; target_now = 0.0f; break;
    case 'x': stopOutput("user stop"); break;
    case 'g': printStatus(); reportPwmRegs(); break;
    default:  Serial1.println("ERR: t/v/p/i/f/s/x/g/o/c"); break;
  }
}

static void pollSerial()
{
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmd_buf[cmd_len] = '\0';
      if (cmd_len) execCommand(cmd_buf);
      cmd_len = 0;
    } else if (cmd_len < sizeof(cmd_buf) - 1) {
      cmd_buf[cmd_len++] = c;
    }
  }
}

/* ------------------------------- setup/loop ------------------------------ */
void setup()
{
  Serial1.begin(115200);
  delay(200);
  Serial1.println("\n=== 2804 BLDC FOC v4 ===");

  /* 1. 编码器：100kHz（原理图只有 10k 上拉，400kHz 太勉强） */
  sensor.init();
  Wire.setClock(100000L);
  delay(10);

  Serial1.print("[SENSOR] angle=");
  Serial1.print(sensor.getAngle() * 180.0f / PI, 1);
  Serial1.println(" deg");

  /* 2. 自写 TIM2 20kHz */
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit       = SUPPLY_VOLTAGE;
  if (!driver.init()) {
    Serial1.println("[PWM] TIM2 init FAILED");
    while (true) delay(10);
  }
  reportPwmRegs();

  /* 3. I2C 空闲自检（此时功率级还没工作） */
  i2cSelfTest();

  /* 4. 50% 直通，示波器量载波 */
  pwmCarrierTest();

  /* 5. 电机环参数 */
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  motor.torque_controller    = TorqueControlType::voltage;
  motor.foc_modulation       = FOCModulationType::SinePWM;
  motor.controller           = MotionControlType::velocity;
  motor.voltage_limit        = closed_vlim;
  motor.voltage_sensor_align = ALIGN_VOLTAGE;
  motor.velocity_limit       = MAX_VELOCITY;
  motor.PID_velocity.P       = 0.6f;
  motor.PID_velocity.I       = 5.0f;
  motor.PID_velocity.D       = 0.0f;
  motor.LPF_velocity.Tf      = 0.01f;

  if (!motor.init()) {
    Serial1.println("[INIT] motor.init() FAILED");
    while (true) delay(10);
  }
  motor.disable();

  /* 6. 开环 -> 对齐 -> 闭环 */
  delay(300);
  openLoopPretest(2.0f, OPENLOOP_TEST_MS);
  sensor.clearStats();
  startClosedLoop(START_VELOCITY);
}

void loop()
{
  pollSerial();

  if (mode == Mode::IDLE) { delay(2); return; }

  static uint32_t last_us = micros();
  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.002f;

  float step = VEL_RAMP * dt;
  float err  = target_set - target_now;
  if      (err >  step) target_now += step;
  else if (err < -step) target_now -= step;
  else                  target_now  = target_set;

  motor.loopFOC();
  motor.move(target_now);

  /* 编码器持续误码 -> 停机，绝不用错角度硬拉 */
  if (sensor.glitch > 40) {
    Serial1.print("[ERR] encoder glitch="); Serial1.println(sensor.glitch);
    stopOutput("encoder unstable");
    return;
  }

  if (mode == Mode::CLOSED_LOOP && fabs(target_now) > 1.5f &&
      fabs(motor.shaft_velocity) < 0.15f * fabs(target_now)) {
    stall_ms += (uint32_t)(dt * 1000.0f);
    if (stall_ms > STALL_TIME_MS) stopOutput("stall");
  } else {
    stall_ms = 0;
  }

  static uint32_t tele_ms = 0;
  tele_ms += (uint32_t)(dt * 1000.0f);
  if (tele_ms >= 200) { tele_ms = 0; printStatus(); }
}
