#pragma once

#include "common-defines.h"

#define PACKET_DATA_LENGHT  (16U)
#define PACKET_LENGHT_BYTES (1U)
#define PACKET_CRC_BYTES    (1U)
#define PACKET_LENGHT       (PACKET_LENGHT_BYTES + PACKET_DATA_LENGHT + PACKET_CRC_BYTES)

#define PACKET_RETX_DATA0   (0X19)
#define PACKET_ACK_DATA0    (0X15)

#define BL_PACKET_SYNC_OBSERVED_DATA0      (0x20)
#define BL_PACKET_FW_UPDATE_REQ_DATA0     (0x31)
#define BL_PACKET_FW_UPDATE_RES_DATA0     (0x37)
#define BL_PACKET_DEVICE_ID_REQ_DATA0     (0x3C)
#define BL_PACKET_DEVICE_ID_RES_DATA0     (0x3F)
#define BL_PACKET_FW_LENGTH_REQ_DATA0     (0x42)
#define BL_PACKET_FW_LENGTH_RES_DATA0     (0x45)
#define BL_PACKET_READY_FOR_DATA_DATA0    (0x48)
#define BL_PACKET_UPDATE_SUCCESSFUL_DATA0 (0x54)
#define BL_PACKET_NACK_DATA0              (0x59)

typedef struct comms_packet_t
{
    uint8_t lenght;
    uint8_t data[PACKET_DATA_LENGHT];
    uint8_t crc;
} comms_packet_t;

void comms_setup(void);
void comms_update(void);
bool comms_packet_available(void);
void comms_write(comms_packet_t* packet);
void comms_read(comms_packet_t* packet);
uint8_t comms_compute_crc(comms_packet_t* packet);
void comms_create_single_byte_packet(comms_packet_t* packet, const uint8_t byte);
bool comms_is_single_byte_packet(const comms_packet_t* packet, const uint8_t byte);

