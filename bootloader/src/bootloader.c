#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/memorymap.h>

#include "common-defines.h"

#define BOOTLOADER_SIZE (0x8000U) // 32KB
#define MAIN_APP_START_ADDRESS (FLASH_BASE + BOOTLOADER_SIZE)

static void jump_to_main_firmware(void);

static void jump_to_main_firmware(void)
{
    vector_table_t* main_vector_table = (vector_table_t*)(MAIN_APP_START_ADDRESS);
    main_vector_table->reset();
}

int main(void)
{
    jump_to_main_firmware();

    // Never return
    return 0;
}