#include <stm32f10x.h>
#include "usart.h"
#include "timer.h"
#include "utils.h"
#include "iic.h"
#include "test.h"
#include "simple_foc.h"

volatile uint16_t sysCnt;

/* 100us 中断节拍, 供 FOC 调度 */
void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_UIF)
    {
        TIM4->SR &= ~TIM_SR_UIF;
        sysCnt++;
    }
}

int main(void)
{
    USART_Init1();
    Timer2_Init();      /* TIM2: 3 相 20 kHz PWM (PA0/1/2) */
    Timer4_Init();      /* TIM4: 100us 定时节拍 */
    IIC_Init();
    IO_Init();          /* PA4/5/6: 半桥使能 GPIO */
    sin_generate(sin_table, SIN_TABLE_SIZE);

    SimpleFOC_Init();
    SimpleFOC_Enable();

    /* ======================================================
     * 两种工作模式二选一 (可在运行时通过上位机/按键切换):
     * ------------------------------------------------------
     *  1) 速度控制 (示例: 20 rad/s ≈ 191 RPM @ 7 极对数):
     *     SimpleFOC_SetTargetSpeed(20.0f);
     *
     *  2) 位置控制 (示例: 转到相对上电位置 90° 处):
     *     SimpleFOC_SetTargetPosition(90.0f);
     *     （支持多圈, 例如 SetTargetPosition(720.0f) = 转 2 圈）
     * ======================================================*/
    SimpleFOC_SetTargetSpeed(60.0f);
    // SimpleFOC_SetTargetPosition(90.0f);

    while (1)
    {
        /* 捕获 100us 节拍数, 追赶丢失的 tick (保证外环 cadence) */
        if (sysCnt > 0)
        {
            __disable_irq();
            uint16_t n = sysCnt; sysCnt = 0;
            __enable_irq();
            while (n--) SimpleFOC_Run100us();
        }
    }

    return 0;
}
