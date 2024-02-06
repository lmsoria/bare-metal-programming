#pragma once

#include "common-defines.h"

#define CPU_FREQ_HZ (96000000U)
#define SYSTICK_FREQ_HZ (1000U)

void system_setup(void);

uint64_t system_get_ticks(void);