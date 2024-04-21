#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include <core/simple_timer.h>
#include <core/system.h>
#include <core/uart.h>
#include "bl_flash.h"
#include "comms.h"
#include "common-defines.h"

#define UART_PORT (GPIOD)
#define UART_TX_PIN (GPIO8)
#define UART_RX_PIN (GPIO9)

#define BOOTLOADER_SIZE (0x8000U) // 32KB
#define MAIN_APP_START_ADDRESS (FLASH_BASE + BOOTLOADER_SIZE)

static void gpio_setup(void);
static void jump_to_main_firmware(void);

int main(void)
{
    system_setup();
    // gpio_setup();
    // uart_setup();
    // comms_setup();

    simple_timer_t timer;
    simple_timer_setup(&timer, 1000, true);

    while (true) {

        if(simple_timer_has_elaped(&timer)) {
            volatile int x = 0;
            x++;
        }

    }

    // TODO: Teardown

    jump_to_main_firmware();

    // Never return
    return 0;
}

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, (UART_TX_PIN | UART_RX_PIN));
    gpio_set_af(UART_PORT, GPIO_AF7, (UART_TX_PIN | UART_RX_PIN));
}


static void jump_to_main_firmware(void)
{
    vector_table_t* main_vector_table = (vector_table_t*)(MAIN_APP_START_ADDRESS);
    main_vector_table->reset();
}