#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/rcc.h>

#include "core/system.h"

static volatile uint64_t ticks = 0;

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


void sys_tick_handler(void)
{
    ticks++;
}


void system_setup(void)
{
    rcc_setup();
    systick_setup();
}

uint64_t system_get_ticks(void) { return ticks; }

void system_delay(uint64_t msec)
{
    uint64_t end_time = system_get_ticks() + msec;
    while(system_get_ticks() < end_time);
}