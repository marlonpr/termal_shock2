// thermal_shock.c
#include "termal_shock.h"
#include "state_machine.h"
#include "esp_log.h"

static const char *TAG = "THERMAL_SHOCK";

/* ================= GLOBAL DATA ================= */

thermal_shock_t ts_data;

/* ================= INTERNAL ================= */

void ts_enter_state(ts_state_t next)
{
    ts_data.state = next;

    switch (next) {

        case TS_IDLE:
            ts_data.cycle_count = 0;
            ts_data.mode = TS_MODE_HOT;
            break;

        case TS_INIT:
        case TS_RUNNING:
        case TS_PAUSED:
        case TS_FINISHED:
        case TS_FAULT:
            /* no local action needed */
            break;
    }

    ESP_LOGI(TAG, "TS state -> %d", next);
}

/* ================= PUBLIC API ================= */

void thermal_shock_init(uint32_t max_cycles)
{
    ts_data.state       = TS_INIT;
    ts_data.mode        = TS_MODE_HOT;
    ts_data.cycle_count = 0;
    ts_data.max_cycles  = max_cycles;
    ts_data.init_done   = false;

    ESP_LOGI(TAG, "Thermal shock initialized (max_cycles=%lu)", max_cycles);
}

void thermal_shock_start(void)
{
    if (!ts_data.init_done) {
        ESP_LOGW(TAG, "START rejected: system not armed");
        return;
    }

    if (ts_data.state == TS_IDLE || ts_data.state == TS_FINISHED) {
        sm_start();
        ts_enter_state(TS_RUNNING);
        ESP_LOGI(TAG, "Thermal shock STARTED");
    }
}

void thermal_shock_pause(void)
{
    if (ts_data.state == TS_RUNNING) {
        sm_stop();
        ts_enter_state(TS_PAUSED);
        ESP_LOGI(TAG, "Thermal shock PAUSED");
    }
}

void thermal_shock_reset(void)
{
    sm_stop();
    ts_enter_state(TS_IDLE);

    ESP_LOGI(TAG, "Thermal shock RESET");
}

/* ================= CALLBACKS FROM FSM ================= */

void thermal_shock_notify_hot(void)
{
    ts_data.mode = TS_MODE_HOT;
    ESP_LOGI(TAG, "Mode -> HOT");
}

void thermal_shock_notify_cold(void)
{
    ts_data.mode = TS_MODE_COLD;
    ESP_LOGI(TAG, "Mode -> COLD");
}

void thermal_shock_notify_cycle_complete(void)
{
    ts_data.cycle_count++;

    ESP_LOGI(TAG,
             "Cycle %lu / %lu completed",
             ts_data.cycle_count,
             ts_data.max_cycles);

    if (ts_data.cycle_count >= ts_data.max_cycles) {
        sm_stop();
        ts_enter_state(TS_FINISHED);
        ESP_LOGI(TAG, "Thermal shock FINISHED");
    }
}

