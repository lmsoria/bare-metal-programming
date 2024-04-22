#include <assert.h>
#include <string.h>

#include "core/crc8.h"
#include "core/uart.h"

#include "comms.h"

#define PACKET_BUFFER_LENGTH (8U)

typedef enum comms_state_e {
    COMMS_STATE_LENGHT = 0,
    COMMS_STATE_DATA,
    COMMS_STATE_CRC,
} comms_state_e;

static comms_state_e state = COMMS_STATE_LENGHT;
static uint8_t data_bytes_received = 0;

static comms_packet_t temporary_packet = {.lenght = 0, .data = {0}, .crc = 0 };
static comms_packet_t last_transmitted_packet = {.lenght = 0, .data = {0}, .crc = 0 };
static comms_packet_t retx_packet = {.lenght = 1, .data = {0xFF}, .crc = 0 };
static comms_packet_t ack_packet = {.lenght = 1, .data = {0xFF}, .crc = 0 };

static comms_packet_t packet_buffer[PACKET_BUFFER_LENGTH] = {0};
static uint32_t packet_read_index = 0;
static uint32_t packet_write_index = 0;
static uint32_t packet_buffer_mask = PACKET_BUFFER_LENGTH - 1;

void comms_setup(void)
{
    comms_create_single_byte_packet(&retx_packet, PACKET_RETX_DATA0);
    comms_create_single_byte_packet(&ack_packet, PACKET_ACK_DATA0);
}

void comms_update(void)
{
    while (uart_data_available()) {
        switch (state)
        {
        case COMMS_STATE_LENGHT:
            temporary_packet.lenght = uart_read_byte();
            state = COMMS_STATE_DATA;
            break;
        case COMMS_STATE_DATA:
            temporary_packet.data[data_bytes_received++] = uart_read_byte();
            if(data_bytes_received >= PACKET_DATA_LENGHT) {
                data_bytes_received = 0;
                state = COMMS_STATE_CRC;
            }
            break;
        case COMMS_STATE_CRC:
            temporary_packet.crc = uart_read_byte();

            if(temporary_packet.crc != comms_compute_crc(&temporary_packet)) {
                // We received a corrupt packet, let's ask for the retransmission of it.
                comms_write(&retx_packet);
                state = COMMS_STATE_LENGHT;
                break;
            }

            if(comms_is_single_byte_packet(&temporary_packet, PACKET_RETX_DATA0)) {
                // We received a reTX packet, so let's send the last received packet.
                comms_write(&last_transmitted_packet);
                state = COMMS_STATE_LENGHT;
                break;
            }

            if(comms_is_single_byte_packet(&temporary_packet, PACKET_ACK_DATA0)) {
                // We received an ACK, so don't do anything else.
                state = COMMS_STATE_LENGHT;
                break;
            }

            uint32_t next_write_index = (packet_write_index + 1) & packet_buffer_mask;
            assert(next_write_index != packet_read_index);

            memcpy(&packet_buffer[packet_write_index], &temporary_packet, sizeof(comms_packet_t));
            packet_write_index = next_write_index;
            comms_write(&ack_packet);
            state = COMMS_STATE_LENGHT;

            break;
        default:
            state = COMMS_STATE_LENGHT;
            break;
        }
    }
}

bool comms_packet_available(void) { return (packet_read_index != packet_write_index); }

void comms_write(comms_packet_t* packet)
{
    uart_write((uint8_t*)(packet), PACKET_LENGHT);
    memcpy(&last_transmitted_packet, packet, sizeof(comms_packet_t));
}

void comms_read(comms_packet_t* packet)
{
    memcpy(packet, &packet_buffer[packet_read_index], sizeof(comms_packet_t));
    packet_read_index = (packet_read_index + 1) & packet_buffer_mask;
}

uint8_t comms_compute_crc(comms_packet_t* packet) { return crc8((uint8_t*)(packet), PACKET_LENGHT - PACKET_CRC_BYTES); }


void comms_create_single_byte_packet(comms_packet_t* packet, const uint8_t byte)
{
    memset(packet, 0xFF, sizeof(comms_packet_t));
    packet->lenght = 1;
    packet->data[0] = byte;
    packet->crc = comms_compute_crc(packet);
}

bool comms_is_single_byte_packet(const comms_packet_t* packet, const uint8_t byte)
{
    if(packet->lenght != 1) {
        return false;
    }

    if (packet->data[0] != byte) {
        return false;
    }

    for(uint8_t i = 1; i < PACKET_DATA_LENGHT; i++) {
        if(packet->data[i] != 0xFF) {
            return false;
        }
    }

    return true;
}
