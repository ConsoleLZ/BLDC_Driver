#include "tim2_driver.h"

/* 构造函数：交给基类保存引脚，PWM 参数在 init() 中配置 */
Tim2Driver::Tim2Driver()
    : BLDCDriver3PWM(PIN_PWM_U, PIN_PWM_V, PIN_PWM_W,
                     PIN_EN_U, PIN_EN_V, PIN_EN_W) {}

/* 初始化 GPIO 与 TIM2：中心对齐 + 互补不需要，仅三通道独立输出 */
int Tim2Driver::init()
{
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

/* 使能 DRV8313 并输出零占空比 */
void Tim2Driver::enable()
{
  GPIOA->BSRR = (1UL << 4) | (1UL << 5) | (1UL << 6);
  setPwm(0, 0, 0);
}

/* 关闭前先把占空比清零，避免关断瞬间残留电压 */
void Tim2Driver::disable()
{
  setPwm(0, 0, 0);
  GPIOA->BRR = (1UL << 4) | (1UL << 5) | (1UL << 6);
}

/* 电压 -> 占空比 -> 比较寄存器，电压限制由基类成员给出 */
void Tim2Driver::setPwm(float Ua, float Ub, float Uc)
{
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
