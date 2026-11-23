#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/* ============================================================================
 * 2804 无刷电机(12N14P, 7 对极) + DRV8313 + AS5600 — 上电自动运行 v8（静默版）
 *
 * 已删掉全部串口打印 / 自检 / 噪声测试 / 串口命令解析，只保留控制逻辑。
 * 启动流程：上电 -> (可选) FOC 对齐 -> 直接按斜坡冲到目标转速（无暖机、无预检）。
 *          参数标定完成后上电不再对齐，电机完全不动直接起转。
 * 本文件集中存放全部宏定义与可调参数，改转速 / 引脚只需改这里。
 *
 * 【改转速看这里】
 *   STAGE2_VELOCITY  —— 目标转速（默认 20.94 rad/s ≈ 200 rpm）★主要改这个
 *   VEL_RAMP         —— 加速度 rad/s²，越大起步越猛（20.94 / VEL_RAMP ≈ 起转时间）
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
#define SUPPLY_VOLTAGE 12.0f
#define POLE_PAIRS 7
#define PHASE_RESISTANCE 2.55f
#define MOTOR_KV 220.0f
#define PHASE_INDUCTANCE 0.00086f

/* ------------------------------- 引脚定义 -------------------------------- */
#define PIN_PWM_U PA0
#define PIN_PWM_V PA1
#define PIN_PWM_W PA2
#define PIN_EN_U PA4
#define PIN_EN_V PA5
#define PIN_EN_W PA6

/* ------------------------------- 运行参数 -------------------------------- */
#define PWM_FREQ_HZ 20000UL
#define ALIGN_VOLTAGE 2.5f
#define OPENLOOP_VOLTAGE 2.0f
/**
 * 转速升高后反电动势（Ke = 0.0434 V/(rad/s)）会吃掉电压：稳态需要 `U ≈ Ke·ω + I·R`。2.5 V 幅度下理论空载极限约 `2.5/0.0434 ≈ 57 rad/s`（≈550 rpm），
 * 但带载和摩擦会先撞墙。想明显提速就把 `CLOSED_VOLTAGE` 一起抬（12 V 供电下建议不超过 4.0 V，否则电流和发热上来，2.55 Ω / 0.5 A 额定的线圈扛不住）。
 */
#define CLOSED_VOLTAGE 2.5f

#define STAGE2_VELOCITY 50.0f // ★目标转速 rad/s（改这里）
/**
 * 改速度时别忘它得 ≥ 你的目标。另外闭环启动时速度环会按 `STAGE2_VELOCITY` 直接算出 q 轴电压，如果设太大而 `CLOSED_VOLTAGE` 不够，就会卡在限幅上失速
 */
#define MAX_VELOCITY 40.0f
#define VEL_RAMP 100.0f // 加速度 rad/s²（100 -> 约 0.2s 到目标转速）

#define STALL_TIME_MS 3000

/* ----------------------------- 电机对齐标定 ------------------------------
 * 抖动机理：motor.initFOC() 默认会让电机正转一圈电角度再倒转一圈（约 2 秒），
 * 7 对极电机对应机械角约 ±51°，所以上电会看到明显摆动。
 *
 * 首次烧录：保持 ALIGN_CALIBRATE = 1，上电会做一次完整对齐，并通过
 *   串口1(PA9/PA10, 115200) 输出两行：
 *     [CAL] ZERO= x.xxxxx
 *     [CAL] DIR= 1
 * 把这两个值分别填入下面两项，再把 ALIGN_CALIBRATE 改成 0 重新烧录。
 * 之后 initFOC() 会跳过全部对齐动作，上电零抖动、立即加速到目标转速。
 * ----------------------------------------------------------------------*/
#define ALIGN_CALIBRATE 0
#define MOTOR_ZERO_ELECTRIC_ANGLE 1.1f // rad，填标定输出的 ZERO
#define MOTOR_SENSOR_DIRECTION -1      // 1=CW，-1=CCW，填标定输出的 DIR

/* 速度环：J=3.7e-6kg·m²，带宽≈4600·P=230rad/s(37Hz)，PI零点40rad/s */
#define VEL_P 0.05f
#define VEL_I 2.0f
#define VEL_TF 0.002f

/* ----------------------------- 内部保护阈值 ------------------------------ */
#define OPENLOOP_VEL_LIMIT 10.0f // 开环目标速度限幅 rad/s
#define ENC_GLITCH_STOP 40       // 闭环中连续误码停机阈值
#define STALL_VEL_RATIO 0.15f    // 实际转速低于目标的该比例视为堵转
#define STALL_VEL_MIN 1.0f       // 堵转检测生效的最小目标转速 rad/s
#define LOOP_DT_DEFAULT 0.002f   // dt 异常时的兜底值 s
#define LOOP_DT_MAX 0.05f        // dt 上限 s

#endif // CONFIG_H
