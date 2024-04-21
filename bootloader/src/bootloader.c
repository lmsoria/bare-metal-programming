#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

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


    uint8_t data[1024] = {0};
    for(uint16_t i = 0; i < 1024; i++) {
        data[i] = i & 0xFF;
    }

    bl_flash_erase_main_application();
    bl_flash_write(0x08008000, data, 1024);
    bl_flash_write(0x0800C000, data, 1024);
    bl_flash_write(0x08010000, data, 1024);
    bl_flash_write(0x08020000, data, 1024);
    bl_flash_write(0x08040000, data, 1024);
    bl_flash_write(0x08060000, data, 1024);
    bl_flash_write(0x08080000, data, 1024);
    bl_flash_write(0x080A0000, data, 1024);
    bl_flash_write(0x080C0000, data, 1024);
    bl_flash_write(0x080E0000, data, 1024);
    bl_flash_write(0x08100000, data, 1024);
    bl_flash_write(0x08120000, data, 1024);
    bl_flash_write(0x08140000, data, 1024);
    bl_flash_write(0x08160000, data, 1024);

    while (true) {

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