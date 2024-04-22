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
#define MAX_FW_LENGTH ((1024U * 512U) - BOOTLOADER_SIZE)

#define DEVICE_ID (0x42)

#define SYNC_SEQ_0 (0xC4)
#define SYNC_SEQ_1 (0x55)
#define SYNC_SEQ_2 (0x7E)
#define SYNC_SEQ_3 (0x10)

#define DEFAULT_TIMEOUT_MS (10000U)

typedef enum bl_state_e {
    BL_STATE_SYNC = 0,
    BL_STATE_WAITING_FOR_UPDATE_REQUEST,
    BL_STATE_DEVICE_ID_REQUEST,
    BL_STATE_DEVICE_ID_RESPONSE,
    BL_STATE_FW_LENGTH_REQUEST,
    BL_STATE_FW_LENGTH_RESPONSE,
    BL_STATE_ERASE_APPLICATION,
    BL_STATE_RECEIVE_FW,
    BL_STATE_DONE,
} bl_state_e;

static bl_state_e state = BL_STATE_SYNC;
static uint32_t fw_length = 0;
static uint32_t bytes_written = 0;
static uint8_t sync_seq[4] = {0};

static simple_timer_t timer;
static comms_packet_t temp_packet;

static void gpio_setup(void);
static void gpio_deinit(void);
static void jump_to_main_firmware(void);

static void check_for_timeout(void);
static void send_nack(void);

static bool is_device_id_packet(const comms_packet_t* const packet);
static bool is_firmware_length_packet(const comms_packet_t* const packet);

int main(void)
{
    system_setup();
    gpio_setup();
    uart_setup();
    comms_setup();

    simple_timer_setup(&timer, DEFAULT_TIMEOUT_MS, false);

    while(state != BL_STATE_DONE) {

        if(state == BL_STATE_SYNC) {
            if(uart_data_available()) {
                sync_seq[0] = sync_seq[1];
                sync_seq[1] = sync_seq[2];
                sync_seq[2] = sync_seq[3];
                sync_seq[3] = uart_read_byte();

                bool is_match = (sync_seq[0] = SYNC_SEQ_0);
                is_match = (sync_seq[1] = SYNC_SEQ_1);
                is_match = (sync_seq[2] = SYNC_SEQ_2);
                is_match = (sync_seq[3] = SYNC_SEQ_3);

                if(is_match) {
                    comms_create_single_byte_packet(&temp_packet, BL_PACKET_SYNC_OBSERVED_DATA0);
                    comms_write(&temp_packet);
                    simple_timer_reset(&timer);
                    state = BL_STATE_WAITING_FOR_UPDATE_REQUEST;
                } else {
                    check_for_timeout();
                }
            } else {
                check_for_timeout();
            }

            continue;
        }

        comms_update();

        switch (state)
        {
        case BL_STATE_WAITING_FOR_UPDATE_REQUEST: {
            if(comms_packet_available()) {
                comms_read(&temp_packet);

                if(comms_is_single_byte_packet(&temp_packet, BL_PACKET_FW_UPDATE_REQ_DATA0)) {
                    simple_timer_reset(&timer);
                    comms_create_single_byte_packet(&temp_packet, BL_PACKET_FW_UPDATE_RES_DATA0);
                    comms_write(&temp_packet);
                    state = BL_STATE_DEVICE_ID_REQUEST;
                } else {
                    send_nack();
                }
            } else {
                check_for_timeout();
            }
            break;
        }

        case BL_STATE_DEVICE_ID_REQUEST: {
            simple_timer_reset(&timer);
            comms_create_single_byte_packet(&temp_packet, BL_PACKET_DEVICE_ID_REQ_DATA0);
            comms_write(&temp_packet);
            state = BL_STATE_DEVICE_ID_REQUEST;
            break;
        }

        case BL_STATE_DEVICE_ID_RESPONSE: {
            if(comms_packet_available()) {
                comms_read(&temp_packet);

                if(is_device_id_packet(&temp_packet) && temp_packet.data[1] == DEVICE_ID) {
                    simple_timer_reset(&timer);
                    state = BL_STATE_FW_LENGTH_REQUEST;
                } else {
                    send_nack();
                }
            } else {
                check_for_timeout();
            }
            break;
        }

        case BL_STATE_FW_LENGTH_REQUEST: {
            simple_timer_reset(&timer);
            comms_create_single_byte_packet(&temp_packet, BL_PACKET_FW_LENGTH_REQ_DATA0);
            comms_write(&temp_packet);
            state = BL_STATE_FW_LENGTH_RESPONSE;
            break;
        }

        case BL_STATE_FW_LENGTH_RESPONSE: {
            if(comms_packet_available()) {
                comms_read(&temp_packet);
                if(is_firmware_length_packet(&temp_packet)) {
                    fw_length = (
                        temp_packet.data[1] << 0  |
                        temp_packet.data[2] << 8  |
                        temp_packet.data[3] << 16 |
                        temp_packet.data[4] << 24
                    );
                    if(fw_length <= MAX_FW_LENGTH) {
                        simple_timer_reset(&timer);
                        state = BL_STATE_ERASE_APPLICATION;
                    } else {
                        send_nack();
                    }
                } else {
                    send_nack();
                }
            } else {
                check_for_timeout();
            }
            break;
        }

        case BL_STATE_ERASE_APPLICATION: {
            bl_flash_erase_main_application();
            simple_timer_reset(&timer);
            state = BL_STATE_RECEIVE_FW;
            break;
        }

        case BL_STATE_RECEIVE_FW: {
            if(comms_packet_available()) {
                comms_read(&temp_packet);

                const uint8_t PACKET_LENGTH = (temp_packet.lenght & 0x0F) + 1;
                bl_flash_write(MAIN_APP_START_ADDRESS + bytes_written, temp_packet.data, PACKET_LENGTH);
                bytes_written += PACKET_LENGTH;
                simple_timer_reset(&timer);

                if(bytes_written >= fw_length) {
                    comms_create_single_byte_packet(&temp_packet, BL_PACKET_UPDATE_SUCCESSFUL_DATA0);
                    comms_write(&temp_packet);
                    state = BL_STATE_DONE;
                }

            } else {
                check_for_timeout();
            }
            break;
        }

        default:
            state = BL_STATE_SYNC;
            break;
        }

    }

    system_delay(200);

    uart_deinit();
    gpio_deinit();
    system_deinit();

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

static void gpio_deinit(void)
{
    gpio_mode_setup(UART_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, (UART_TX_PIN | UART_RX_PIN));
    rcc_periph_clock_disable(RCC_GPIOD);
}


static void jump_to_main_firmware(void)
{
    vector_table_t* main_vector_table = (vector_table_t*)(MAIN_APP_START_ADDRESS);
    main_vector_table->reset();
}

static void check_for_timeout(void)
{
    if(simple_timer_has_elaped(&timer)) {
        send_nack();
        state = BL_STATE_DONE;
    }
}

static void send_nack(void)
{
    comms_create_single_byte_packet(&temp_packet, BL_PACKET_NACK_DATA0);
    comms_write(&temp_packet);
}

static bool is_device_id_packet(const comms_packet_t* const packet)
{
    if(packet->lenght != 2) {
        return false;
    }

    if (packet->data[0] != BL_PACKET_DEVICE_ID_RES_DATA0) {
        return false;
    }

    for(uint8_t i = 2; i < PACKET_DATA_LENGHT; i++) {
        if(packet->data[i] != 0xFF) {
            return false;
        }
    }

    return true;
}

static bool is_firmware_length_packet(const comms_packet_t* const packet)
{
    if(packet->lenght != 5) {
        return false;
    }

    if (packet->data[0] != BL_PACKET_FW_LENGTH_RES_DATA0) {
        return false;
    }

    for(uint8_t i = 5; i < PACKET_DATA_LENGHT; i++) {
        if(packet->data[i] != 0xFF) {
            return false;
        }
    }

    return true;
}