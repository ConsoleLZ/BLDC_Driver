#include "timer.h"
#include "test.h"

void Timer8_Init(void)
{
    // 开启时钟
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    /* 设置 GPIO 为复用推挽输出（PC6, PC7, PC8） */
    // PC6: CNF6=10, MODE6=11
    GPIOC->CRL |= (GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6);
    GPIOC->CRL &= ~GPIO_CRL_CNF6_0;
    // PC7: CNF7=10, MODE7=11
    GPIOC->CRL |= (GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7);
    GPIOC->CRL &= ~GPIO_CRL_CNF7_0;
    // PC8: CNF8=10, MODE8=11
    GPIOC->CRH |= (GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8);
    GPIOC->CRH &= ~GPIO_CRH_CNF8_0;

    /* 定时器配置 */
    // 72MHz / 3600 = 20kHz PWM，避开可听频段
    TIM8->PSC = 0;
    TIM8->ARR = PWM_PERIOD - 1;
    // 计数方向：向上
    TIM8->CR1 &= ~TIM_CR1_DIR;

    // 配置通道1 的捕获比较寄存器（占空比）
    TIM8->CCR1 = 0;
    // 通道1 配置为输出
    TIM8->CCMR1 &= ~TIM_CCMR1_CC1S;
    // 输出比较模式：PWM1 (110)
    TIM8->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM8->CCMR1 &= ~TIM_CCMR1_OC1M_0;
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE;
    // 使能通道1
    TIM8->CCER |= TIM_CCER_CC1E;
    // 极性：高电平有效（0）
    TIM8->CCER &= ~TIM_CCER_CC1P;

    // 配置通道2
    TIM8->CCR2 = 0;
    TIM8->CCMR1 &= ~TIM_CCMR1_CC2S;
    TIM8->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM8->CCMR1 &= ~TIM_CCMR1_OC2M_0;
    TIM8->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM8->CCER |= TIM_CCER_CC2E;
    TIM8->CCER &= ~TIM_CCER_CC2P;

    // 配置通道3（使用 CCMR2 寄存器）
    TIM8->CCR3 = 0;
    TIM8->CCMR2 &= ~TIM_CCMR2_CC3S;
    TIM8->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
    TIM8->CCMR2 &= ~TIM_CCMR2_OC3M_0;
    TIM8->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM8->CCER |= TIM_CCER_CC3E;
    TIM8->CCER &= ~TIM_CCER_CC3P;

    // CCR/ARR 在更新事件同时生效，避免三相占空比更新不同步
    TIM8->CR1 |= TIM_CR1_ARPE;
    TIM8->EGR = TIM_EGR_UG;

    // 高级定时器必须使能主输出（MOE）
    TIM8->BDTR |= TIM_BDTR_MOE;

    // 使能计数器
    TIM8->CR1 |= TIM_CR1_CEN;
}

// 100us
void Timer6_Init(void)
{
    // 开启定时器6的时钟
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // 设置预分频值: 分频值7199表示7200分频。
    TIM6->PSC = 72 - 1;

    TIM6->ARR = 100 - 1;

    // 使能更新中断
    TIM6->DIER |= TIM_DIER_UIE;

    // 设置优先级
    NVIC_SetPriorityGrouping(3);
    NVIC_SetPriority(TIM6_IRQn, 1);
    NVIC_EnableIRQ(TIM6_IRQn);

    // 使能计数器
    TIM6->CR1 |= TIM_CR1_CEN;
}
