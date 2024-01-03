#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define LED_PORT (GPIOB)
#define LED_PIN  (GPIO0)
#define CPU_FREQ_HZ (96000000U)
#define SYSTICK_FREQ_HZ (1000U)

static volatile uint64_t ticks = 0;

void sys_tick_handler(void)
{
    ticks++;
}

static uint64_t get_ticks(void) { return ticks; }

static void rcc_setup(void)
{
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_96MHZ]);
}

static void systick_setup(void)
{
    systick_set_frequency(SYSTICK_FREQ_HZ, CPU_FREQ_HZ);
    systick_counter_enable();
    systick_interrupt_enable();
}

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}



int main(void)
{
    rcc_setup();
    gpio_setup();

    systick_setup();

    uint64_t start_time = get_ticks();

    while(1) {
        if(get_ticks() - start_time >= 100) {
            gpio_toggle(LED_PORT, LED_PIN);
            start_time = get_ticks();
        }

        // Do useful work
    }

    // Never return
    return 0;
}