#include "as5600.h"
#include "iic.h"

uint8_t AS5600_TryReadRawAngle(uint16_t *raw_angle)
{
    uint8_t high, low;

    if (!IIC_Start()) goto error;

    if (!IIC_SendAddr(AS5600_ADDR << 1)) goto error;

    if (!IIC_SendByte(AS5600_RAW_ANGLE_HIGH)) goto error;

    if (!IIC_Start()) goto error;

    /* F1 必须在清除读地址的 ADDR 标志前打开 ACK，否则首字节可能被 NACK。 */
    IIC_Ack();

    if (!IIC_SendAddr((AS5600_ADDR << 1) | 0x01)) goto error;

    if (!IIC_ReadByte(&high)) goto error;

    IIC_Nck();

    if (!IIC_ReadByte(&low)) goto error;

    IIC_Stop();

    *raw_angle = (((uint16_t)high << 8) | low) & 0x0FFF;

    return 1;

error:
    IIC_Nck();
    IIC_Stop();
    return 0;
}

uint16_t AS5600_ReadRawAngle(void)
{
    uint16_t raw_angle = 0;

    (void)AS5600_TryReadRawAngle(&raw_angle);
    return raw_angle;
}

uint16_t AS5600_ReadAngle(void)
{
    return (uint16_t)(AS5600_ReadRawAngle() * 3600 / 4096);
}

/* ===== 转速测量 ===== */

#define AS5600_COUNTS_PER_REV   4096        // 编码器一圈的计数
#define AS5600_HALF_REV_COUNTS  2048        // 半圈计数，用于判断环形过零方向
#define AS5600_SPEED_WINDOW_MS  1000U       // 测速窗口

static uint16_t s_last_raw = 0;       // 上一次采样的原始角度
static uint32_t s_last_cycle = 0;     // 上一次采样时的 DWT 周期计数
static uint8_t  s_has_last = 0;       // 是否已有上一次采样
static uint32_t s_win_cycle_max = 0;  // 一个测速窗口对应的周期数
static uint32_t s_win_cycle = 0;      // 当前窗口已累计的周期数
static int32_t  s_win_counts = 0;     // 当前窗口已累计的角度计数
static float    s_speed_rpm = 0.0f;   // 最近一次算出的平均转速

/**
 * @brief 获取当前转速
 * @return float 转速，单位 RPM，正数表示原始角度递增方向
 */
float AS5600_GetSpeedRPM(void)
{
    uint16_t raw;
    uint32_t now_cycle;
    uint32_t dt_cycle;
    int32_t delta;

    // 首次调用：开启 DWT 周期计数器并记录基准角度，本次返回 0
    if (!s_has_last)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        s_win_cycle_max = SystemCoreClock / 1000U * AS5600_SPEED_WINDOW_MS;

        if (!AS5600_TryReadRawAngle(&raw))
        {
            return 0.0f;
        }

        s_last_raw = raw;
        s_last_cycle = DWT->CYCCNT;
        s_has_last = 1;

        return 0.0f;
    }

    // 读失败：保留上次结果，也不更新基准，失败期间的时间累计到下次成功采样
    if (!AS5600_TryReadRawAngle(&raw))
    {
        return s_speed_rpm;
    }

    now_cycle = DWT->CYCCNT;
    dt_cycle = now_cycle - s_last_cycle;
    if (dt_cycle == 0)
    {
        return s_speed_rpm;
    }

    // 12 位角度的环形差分：差值超过半圈说明跨过了 0/4095 边界
    delta = (int32_t)raw - (int32_t)s_last_raw;
    if (delta > AS5600_HALF_REV_COUNTS)
    {
        delta -= AS5600_COUNTS_PER_REV;
    }
    else if (delta < -AS5600_HALF_REV_COUNTS)
    {
        delta += AS5600_COUNTS_PER_REV;
    }

    s_last_raw = raw;
    s_last_cycle = now_cycle;

    /*
     * 单次调用只间隔十几毫秒，覆盖不到一个完整电周期，转矩脉动会直接
     * 体现在瞬时值上；累计一个测速窗口后再换算，得到真实平均转速。
     */
    s_win_counts += delta;
    s_win_cycle += dt_cycle;

    if (s_win_cycle >= s_win_cycle_max)
    {
        // RPM = (counts / 4096) 圈 ÷ (cycle / 时钟频率) 秒 × 60
        s_speed_rpm = (float)s_win_counts * 60.0f * (float)SystemCoreClock /
                      ((float)AS5600_COUNTS_PER_REV * (float)s_win_cycle);

        s_win_counts = 0;
        s_win_cycle = 0;
    }

    return s_speed_rpm;
}