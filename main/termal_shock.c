// thermal_shock.c
#include "termal_shock.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
        ts_data.timer_sec  = 0;
        ts_data.cycle_count = 0;
        ts_data.mode       = TS_MODE_HOT;
        break;

    case TS_INIT:
        ts_data.timer_sec = 0;
        break;

    case TS_RUNNING:
        ts_data.timer_sec = 0;
        break;

    case TS_PAUSED:
        /* keep timer as-is */
        break;

    case TS_FINISHED:
        ts_data.timer_sec = 0;
        break;

    case TS_FAULT:
        break;
    }

    ESP_LOGI(TAG, "System state -> %d", next);
}

/* ================= PUBLIC API ================= */

void thermal_shock_init(uint32_t max_cycles)
{
    ts_data.state       = TS_INIT;
    ts_data.mode        = TS_MODE_HOT;
    ts_data.cycle_count = 0;
    ts_data.max_cycles  = max_cycles;
    ts_data.timer_sec   = 0;
    ts_data.init_done   = false;

    ESP_LOGI(TAG, "Thermal shock initialized (cycles=%lu)", max_cycles);
}

void thermal_shock_start(void)
{
    if (!ts_data.init_done) {
        ESP_LOGW(TAG, "Start rejected: init not complete");
        return;
    }

    if (ts_data.state == TS_IDLE || ts_data.state == TS_FINISHED) {
        sm_start();
        ts_enter_state(TS_RUNNING);
        ESP_LOGI(TAG, "Thermal shock START");
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
    ts_data.cycle_count = 0;
    ts_data.timer_sec   = 0;
    ts_enter_state(TS_IDLE);

    ESP_LOGI(TAG, "Thermal shock RESET");
}

/* ================= PERIODIC TICK ================= */
/* Call once per second */

void thermal_shock_tick(void)
{
    if (ts_data.state == TS_RUNNING) {
        ts_data.timer_sec++;
    }
}

/* ================= CALLBACKS FROM LOWER FSM ================= */

void thermal_shock_notify_hot(void)
{
    ts_data.mode      = TS_MODE_HOT;
    ts_data.timer_sec = 0;

    ESP_LOGI(TAG, "Mode -> HOT");
}

void thermal_shock_notify_cold(void)
{
    ts_data.mode      = TS_MODE_COLD;
    ts_data.timer_sec = 0;

    ESP_LOGI(TAG, "Mode -> COLD");
}

void thermal_shock_notify_cycle_complete(void)
{
    ts_data.cycle_count++;

    ESP_LOGI(TAG, "Cycle %lu completed", ts_data.cycle_count);

    if (ts_data.cycle_count >= ts_data.max_cycles) {
        sm_stop();
        ts_enter_state(TS_FINISHED);
        ESP_LOGI(TAG, "Thermal shock FINISHED");
    }
}

/* ================= TASK (OPTIONAL) ================= */

void task_thermal_shock(void *arg)
{
    while (1) {
        thermal_shock_tick();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
