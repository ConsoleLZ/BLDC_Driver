#include "test.h"
#include "as5600.h"
#include "usart.h"

/* ============ SPWM 正弦表 (Q15, 256点覆盖0~360°) ============ */
#define SIN_TABLE_SIZE  256
// 三相索引偏移 (120°/360° * 256 = 85.3, 取 85 / 171)
#define IDX_OFF_U       0
#define IDX_OFF_V       85
#define IDX_OFF_W       171

static int16_t  sin_table[SIN_TABLE_SIZE];
static uint8_t  sin_table_ready = 0;

typedef enum
{
    MOTOR_WAIT_SENSOR,
    MOTOR_ALIGN,
    MOTOR_RAMP,
    MOTOR_RUN,
    MOTOR_FAULT
} motor_state_t;

static motor_state_t motor_state;
static uint16_t state_time_ms;
static uint16_t electrical_offset;
static uint16_t aligned_raw_angle;
static uint8_t encoder_valid;
static uint8_t sensor_error_count;

/* 用差分方程生成sin表 (不依赖math.h, Q15定点)
 * 修复: 所有中间乘积都用 int32_t 暂存, 计算完毕再 >>15 存入 int16_t
 * 避免 int32_t → int16_t 隐式截断导致的算法完全失效 */
static void sin_table_init(void)
{
    /* 步长 d = 2π/256:  cos(d)≈0.999699,  sin(d)≈0.024541 */
    const int16_t cosd = 32757;    /* 0.999699 * 32768 */
    const int16_t sind = 804;      /* 0.024541 * 32768 */
    int16_t s = 0;
    int16_t c = 32767;
    int32_t s_next, c_next;
    uint16_t i;

    for (i = 0; i < SIN_TABLE_SIZE; i++)
    {
        sin_table[i] = s;
        /* 必须用 int32_t 保存完整乘积, 再统一下移 */
        s_next = (int32_t)s * cosd + (int32_t)c * sind;
        c_next = (int32_t)c * cosd - (int32_t)s * sind;
        s = (int16_t)(s_next >> 15);
        c = (int16_t)(c_next >> 15);
    }
    /* 强制修正关键点, 消除256步累计误差: 0°/90°/180°/270° */
    sin_table[0]   = 0;
    sin_table[64]  = 32767;
    sin_table[128] = 0;
    sin_table[192] = -32767;
    sin_table_ready = 1;
}

/* 查表: 直接传入 0~255 的表索引 */
static int16_t sin_lookup_idx(uint8_t idx)
{
    return sin_table[idx];
}

/* Q15 sin值 → PWM CCR, 范围 0~PWM_MAX_DUTY, 中心对齐 */
static uint16_t sin_to_pwm(int16_t q15sin, uint16_t amplitude)
{
    int32_t tmp = ((int32_t)q15sin * amplitude) >> 15;
    tmp += PWM_CENTER;
    if (tmp < 1) tmp = 1;
    if (tmp >= (int32_t)PWM_PERIOD) tmp = PWM_PERIOD - 1;
    return (uint16_t)tmp;
}

static void pwm_disable(void)
{
    TIM8->CCR1 = 0;
    TIM8->CCR2 = 0;
    TIM8->CCR3 = 0;
}

static void set_stator_voltage(uint16_t electrical_angle, uint16_t amplitude)
{
    uint8_t idx = (uint8_t)(electrical_angle >> 4);

    TIM8->CCR1 = sin_to_pwm(sin_lookup_idx(idx + IDX_OFF_U), amplitude);
    TIM8->CCR2 = sin_to_pwm(sin_lookup_idx(idx + IDX_OFF_V), amplitude);
    TIM8->CCR3 = sin_to_pwm(sin_lookup_idx(idx + IDX_OFF_W), amplitude);
}

static uint16_t mechanical_to_electrical(uint16_t raw_angle)
{
    uint16_t electrical_angle = (uint16_t)(((uint32_t)raw_angle * POLE_PAIRS) & 0x0FFF);

    if (SENSOR_DIRECTION < 0)
    {
        electrical_angle = (uint16_t)(0U - electrical_angle);
    }
    return electrical_angle;
}

static void enter_fault(void)
{
    motor_state = MOTOR_FAULT;
    state_time_ms = 0;
    encoder_valid = 0;
    sensor_error_count = 0;
    pwm_disable();
}

/* ============ 兼容旧接口 ============ */
void motorDriverInit(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    GPIOC->CRL &= ~GPIO_CRL_CNF6;
    GPIOC->CRL |= GPIO_CRL_MODE6;

    GPIOC->CRL &= ~GPIO_CRL_CNF7;
    GPIOC->CRL |= GPIO_CRL_MODE7;

    GPIOC->CRH &= ~GPIO_CRH_CNF8;
    GPIOC->CRH |= GPIO_CRH_MODE8;
}

void testMotorDriver(void)
{
    static uint8_t step;

    U_LOW; V_LOW; W_LOW;
    switch (step)
    {
    case 0: U_HIGH; break;
    case 1: V_HIGH; break;
    case 2: W_HIGH; break;
    default: break;
    }
    if (++step >= 3) step = 0;
}

void testMotorDriver1(void)
{
    uint16_t angle  = AS5600_ReadAngle();
    uint16_t e_angle = (uint16_t)((angle * POLE_PAIRS + 900U) % 3600);
    uint8_t step = e_angle / 1200;

    U_LOW; V_LOW; W_LOW;
    USART_Send_Byte(step);
    switch (step)
    {
    case 0: U_HIGH; break;
    case 1: V_HIGH; break;
    case 2: W_HIGH; break;
    default: break;
    }
}

void testMotorDriverPwm(void)
{
    uint16_t angle   = AS5600_ReadAngle();
    uint16_t e_angle = (uint16_t)((angle * POLE_PAIRS + 900U) % 3600);
    uint8_t  step    = e_angle / 1200;

    TIM8->CCR1 = 0; TIM8->CCR2 = 0; TIM8->CCR3 = 0;
    USART_Send_Byte(step);
    switch (step)
    {
    case 0: TIM8->CCR1 = 450; break;
    case 1: TIM8->CCR2 = 450; break;
    case 2: TIM8->CCR3 = 450; break;
    default: break;
    }
}

void BLDC_SPWM_Init(void)
{
    sin_table_init();
    motor_state = MOTOR_WAIT_SENSOR;
    state_time_ms = 0;
    electrical_offset = 0;
    aligned_raw_angle = 0;
    encoder_valid = 0;
    sensor_error_count = 0;
    pwm_disable();
}

/* ============ 带对准、软启动与通信保护的正弦驱动主函数 ============ */
void BLDC_SPWM_Drive(void)
{
    uint16_t raw_angle;
    uint16_t electrical_angle;
    uint16_t amplitude;

    if (!sin_table_ready) BLDC_SPWM_Init();

    switch (motor_state)
    {
    case MOTOR_WAIT_SENSOR:
        pwm_disable();
        if (++state_time_ms >= SENSOR_BOOT_TIME_MS)
        {
            motor_state = MOTOR_ALIGN;
            state_time_ms = 0;
        }
        break;

    case MOTOR_ALIGN:
        /* 固定 d 轴磁场使转子落在已知电角，随后记录传感器零位。 */
        set_stator_voltage(0, ALIGN_AMPLITUDE);
        if (AS5600_TryReadRawAngle(&raw_angle))
        {
            aligned_raw_angle = raw_angle;
            encoder_valid = 1;
        }
        if (++state_time_ms >= ALIGN_TIME_MS)
        {
            if (!encoder_valid)
            {
                enter_fault();
                break;
            }

            electrical_offset = (uint16_t)(0U - mechanical_to_electrical(aligned_raw_angle));
            motor_state = MOTOR_RAMP;
            state_time_ms = 0;
            sensor_error_count = 0;
        }
        break;

    case MOTOR_RAMP:
    case MOTOR_RUN:
        if (!AS5600_TryReadRawAngle(&raw_angle))
        {
            if (++sensor_error_count >= MAX_SENSOR_ERRORS)
            {
                enter_fault();
            }
            break;
        }

        sensor_error_count = 0;
        electrical_angle = (uint16_t)(mechanical_to_electrical(raw_angle) +
                                      electrical_offset + TORQUE_ADVANCE);
        if (motor_state == MOTOR_RAMP)
        {
            if (state_time_ms < RAMP_TIME_MS)
            {
                amplitude = ALIGN_AMPLITUDE +
                    (uint16_t)(((uint32_t)(RUN_AMPLITUDE - ALIGN_AMPLITUDE) * state_time_ms) /
                               RAMP_TIME_MS);
                state_time_ms++;
            }
            else
            {
                amplitude = RUN_AMPLITUDE;
                motor_state = MOTOR_RUN;
            }
        }
        else
        {
            amplitude = RUN_AMPLITUDE;
        }
        set_stator_voltage(electrical_angle, amplitude);
        break;

    case MOTOR_FAULT:
    default:
        pwm_disable();
        if (++state_time_ms >= FAULT_RETRY_MS)
        {
            motor_state = MOTOR_WAIT_SENSOR;
            state_time_ms = 0;
        }
        break;
    }
}
