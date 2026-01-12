#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* Timestamp (RTC) */
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;

    /* Temperatures */
    float pt100_1;
    float pt100_2;
    float pt100_3;

    /* Digital inputs */
    bool float_1;
    bool float_2;

    /* System flags */
    uint32_t sequence;
} system_data_t;

extern system_data_t g_system_data;
