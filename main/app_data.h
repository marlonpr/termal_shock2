#pragma once
#include <stdint.h>
#include <stdbool.h>

#define NUM_PT100 2  // number of PT100 sensors

typedef struct {
    uint32_t sequence;

    // RTC
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;

    // PT100 sensors
    float pt100[NUM_PT100];

    // Float switches
    bool float_1;
    bool float_2;

    // Add other system data as needed
} system_data_t;

extern system_data_t g_system_data;
