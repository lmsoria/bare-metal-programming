#include "core/crc8.h"

uint8_t crc8(uint8_t* data, uint32_t lenght)
{
    uint8_t crc = 0;

    for(uint32_t i = 0; i < lenght; i++) {
        crc ^= data[i];
        for(uint32_t j = 0; j < 8; j++) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}