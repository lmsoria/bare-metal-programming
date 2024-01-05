#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

#include "core/timer.h"

// 96.000.000
// freq = system_freq / ( (prescaler-1) * (arr-1) )

#define PRESCALER (96)
#define ARR_VALUE (1000)

void timer_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM3);

    // High level timer configuration
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    // Setup PWM mode
    timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);

    // Enable PWM output
    timer_enable_counter(TIM3);
    timer_enable_oc_output(TIM3, TIM_OC3);

    // Setup frequency and resolution
    timer_set_prescaler(TIM3, PRESCALER - 1);
    timer_set_period(TIM3, ARR_VALUE - 1);
}

void timer_pwm_set_duty_cycle(float duty_cycle)
{
    // duty_cycle = (ccr / arr) * 100
    // ccr = (duty_cycle / 100) * arr

    const float raw_value = (float)ARR_VALUE * (duty_cycle / 100.0f);
    timer_set_oc_value(TIM3, TIM_OC3, (uint32_t)raw_value);
}