#pragma once
#include <stdint.h>

#define NUM_PT100 2

typedef struct __attribute__((packed)) {
    uint8_t  fsm_state;     // SM_HOT_DWELL, SM_COLD_DWELL, SM_WAIT, ...
    uint8_t  ts_state;      // TS_RUNNING, TS_PAUSED, TS_FINISHED
    uint8_t  mode;          // HOT / COLD
    uint32_t elapsed_sec;   // RTC based
    uint32_t cycle_count;
} telemetry_status_t;

typedef struct __attribute__((packed)) {
    float pt100[NUM_PT100];
} telemetry_temperature_t;
