#include "timer.h"
#include <stm32f10x.h>

// 100us
void Timer2_Init(void)
{
    // 开启定时器2的时钟
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // 设置预分频值: 分频值7199表示7200分频。
    TIM2->PSC = 72 - 1;

    TIM2->ARR = 100 - 1;

    // 使能更新中断
    TIM2->DIER |= TIM_DIER_UIE;

    // 设置优先级
    NVIC_SetPriorityGrouping(3);
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);

    // 使能计数器
    TIM2->CR1 |= TIM_CR1_CEN;
}