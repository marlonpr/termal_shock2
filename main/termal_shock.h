#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ================= SYSTEM STATES ================= */

#pragma once

#include <stdint.h>
#include <stdbool.h>

void send_cycle(void);

typedef enum {
    TS_IDLE,
    TS_INIT,
    TS_RUNNING,
    TS_PAUSED,
    TS_FINISHED,
    TS_FAULT
} ts_state_t;

typedef enum {
    TS_MODE_HOT,
    TS_MODE_COLD
} ts_mode_t;

/* Existing API */
void thermal_shock_init(uint32_t max_cycles);
void thermal_shock_start(void);
void thermal_shock_pause(void);
void thermal_shock_reset(void);

/* ===== READ-ONLY TELEMETRY ACCESSORS ===== */

ts_state_t thermal_shock_get_state(void);
ts_mode_t  thermal_shock_get_mode(void);
uint32_t   thermal_shock_get_cycle_count(void);
uint32_t   thermal_shock_get_max_cycles(void);

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


/* Hooks from lower FSM */
void thermal_shock_notify_hot(void);
void thermal_shock_notify_cold(void);
void thermal_shock_notify_cycle_complete(void);
void task_thermal_shock(void *arg);
void ts_enter_state(ts_state_t next);
