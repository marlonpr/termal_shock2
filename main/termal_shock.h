#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ================= SYSTEM STATES ================= */

typedef enum {
    TS_IDLE = 0,
    TS_INIT,
    TS_RUNNING,
    TS_PAUSED,
    TS_FINISHED,
    TS_FAULT
} ts_state_t;

typedef enum {
    TS_MODE_HOT = 0,
    TS_MODE_COLD
} ts_mode_t;

/* ================= SYSTEM DATA ================= */

typedef struct {
    ts_state_t state;
    ts_mode_t  mode;

    uint32_t   cycle_count;
    uint32_t   max_cycles;

    uint32_t   timer_sec;
    bool       init_done;
} thermal_shock_t;

/* ================= GLOBAL INSTANCE ================= */

extern thermal_shock_t ts_data;

/* ================= API ================= */

void thermal_shock_init(uint32_t max_cycles);
void thermal_shock_start(void);
void thermal_shock_pause(void);
void thermal_shock_reset(void);

/* Hooks from lower FSM */
void thermal_shock_notify_hot(void);
void thermal_shock_notify_cold(void);
void thermal_shock_notify_cycle_complete(void);
void task_thermal_shock(void *arg);

