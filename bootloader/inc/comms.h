#pragma once

#include "common-defines.h"

#define PACKET_DATA_LENGHT  (16U)
#define PACKET_LENGHT_BYTES (1U)
#define PACKET_CRC_BYTES    (1U)
#define PACKET_LENGHT       (PACKET_LENGHT_BYTES + PACKET_DATA_LENGHT + PACKET_CRC_BYTES)

#define PACKET_RETX_DATA0   (0X19)
#define PACKET_ACK_DATA0    (0X15)

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

