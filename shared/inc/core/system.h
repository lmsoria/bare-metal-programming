#pragma once

#include "common-defines.h"

#define CPU_FREQ_HZ (96000000U)
#define SYSTICK_FREQ_HZ (1000U)

void system_setup(void);
void system_deinit(void);
void system_delay(uint64_t msec);

uint64_t system_get_ticks(void);