#include "timer.h"
#include <stm32f10x.h>

// 100us
void Timer4_Init(void)
{
    // 开启定时器4的时钟
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // 设置预分频值: 分频值71表示72分频。
    TIM4->PSC = 72 - 1;

    TIM4->ARR = 100 - 1;

    // 使能更新中断
    TIM4->DIER |= TIM_DIER_UIE;

    // 设置优先级
    NVIC_SetPriorityGrouping(3);
    NVIC_SetPriority(TIM4_IRQn, 1);
    NVIC_EnableIRQ(TIM4_IRQn);

    // 使能计数器
    TIM4->CR1 |= TIM_CR1_CEN;
}

void Timer2_Init(void)
{
    // 开启时钟
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    /* 设置 GPIO 为复用推挽输出（PA0, PA1, PA2） */
    // PA0: CNF0=10, MODE0=11
    GPIOA->CRL |= (GPIO_CRL_CNF0_1 | GPIO_CRL_MODE0);
    GPIOA->CRL &= ~GPIO_CRL_CNF0_0;
    // PA1: CNF1=10, MODE1=11
    GPIOA->CRL |= (GPIO_CRL_CNF1_1 | GPIO_CRL_MODE1);
    GPIOA->CRL &= ~GPIO_CRL_CNF1_0;
    // PA2: CNF2=10, MODE2=11
    GPIOA->CRL |= (GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2);
    GPIOA->CRL &= ~GPIO_CRL_CNF2_0;

    /* 定时器配置 */
    TIM2->PSC = 0;
    TIM2->ARR = PWM_PERIOD - 1;
    // 计数方向：向上
    TIM2->CR1 &= ~TIM_CR1_DIR;

    // 配置通道1（PA0）的捕获比较寄存器（占空比）
    TIM2->CCR1 = 0;
    // 通道1 配置为输出
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    // 输出比较模式：PWM1 (110)
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M_0;
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
    // 使能通道1
    TIM2->CCER |= TIM_CCER_CC1E;
    // 极性：高电平有效（0）
    TIM2->CCER &= ~TIM_CCER_CC1P;

    // 配置通道2（PA1）
    TIM2->CCR2 = 0;
    TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;
    TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M_0;
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM2->CCER |= TIM_CCER_CC2E;
    TIM2->CCER &= ~TIM_CCER_CC2P;

    // 配置通道3（PA2）（使用 CCMR2 寄存器）
    TIM2->CCR3 = 0;
    TIM2->CCMR2 &= ~TIM_CCMR2_CC3S;
    TIM2->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
    TIM2->CCMR2 &= ~TIM_CCMR2_OC3M_0;
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM2->CCER |= TIM_CCER_CC3E;
    TIM2->CCER &= ~TIM_CCER_CC3P;

    // CCR/ARR 在更新事件同时生效，避免三路占空比更新不同步
    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->EGR = TIM_EGR_UG;

    // 使能计数器
    TIM2->CR1 |= TIM_CR1_CEN;
}