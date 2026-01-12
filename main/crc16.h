#pragma once
#include <stdint.h>
#include <stddef.h>

/* CRC-CCITT (0x1021) */
static inline uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc)
{
    while (len--) {
        crc ^= (*data++) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}
