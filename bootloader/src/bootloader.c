#include <libopencm3/stm32/memorymap.h>

#include "common-defines.h"

#define BOOTLOADER_SIZE (0x8000U) // 32KB
#define MAIN_APP_START_ADDRESS (FLASH_BASE + BOOTLOADER_SIZE)

static void jump_to_main(void);

static void jump_to_main(void)
{
    typedef void (*void_function)(void);

    uint32_t* reset_vector_entry = (uint32_t*)(MAIN_APP_START_ADDRESS + 4U); // First entry of MAIN_APP_START_ADDRESS is the Stack Pointer, so jumpt to the next index to get the Reset Vector
    uint32_t* reset_vector = (uint32_t*)(*reset_vector_entry);

    void_function jump_function = (void_function)(reset_vector);

    jump_function();
}

int main(void)
{
    jump_to_main();

    // Never return
    return 0;
}