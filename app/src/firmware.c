#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>

#include "core/system.h"
#include "core/timer.h"

#define BOOTLOADER_SIZE (0x8000)

#define LED_PORT (GPIOB)
#define LED_PIN  (GPIO0)

static void vector_setup(void)
{
    SCB_VTOR = BOOTLOADER_SIZE;
}

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(LED_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, LED_PIN);
    gpio_set_af(LED_PORT, GPIO_AF2, LED_PIN);
}

int main(void)
{
    vector_setup();
    system_setup();
    gpio_setup();
    timer_setup();

    uint64_t start_time = system_get_ticks();

    float duty_cycle = 0.0f;

    timer_pwm_set_duty_cycle(duty_cycle);

    while(1) {
        if(system_get_ticks() - start_time >= 10) {
            // gpio_toggle(LED_PORT, LED_PIN);
            duty_cycle += 1.0f;
            if(duty_cycle > 100.0f) { duty_cycle = 0.0f; }
            timer_pwm_set_duty_cycle(duty_cycle);
            start_time = system_get_ticks();
        }

        // Do useful work
    }

    // Never return
    return 0;
}