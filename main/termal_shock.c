// thermal_shock.c
#include "termal_shock.h"
#include "protocol.h"
#include "state_machine.h"
#include "esp_log.h"

#include "master_transport.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "board_pins.h"

#include "state_machine.h"
#include "termal_shock.h"
#include "master_transport.h"
#include "master_link.h"
#include "protocol.h"

/* ================= GLOBAL DATA ================= */

void send_cycle(void)
{
    char tx_buf2[32];
	int n2 = snprintf(tx_buf2, sizeof(tx_buf2),
	                 "CYCLES=%lu\n",
	                 (unsigned long)ts_data.cycle_count);
	
	if (n2 > 0) {
	    uart_write_bytes(UART_UI, tx_buf2, n2);
	}
    ESP_LOGI("TX", "Sending: %s", tx_buf2);
}

thermal_shock_t ts_data;

/* ================= INTERNAL ================= */

 void ts_enter_state(ts_state_t next)
{
    ts_data.state = next;

    switch (next) {

    case TS_IDLE:
        ts_data.cycle_count = 0 ;
        /* DO NOT force mode here */
        break;

    case TS_INIT:
    case TS_RUNNING:
    case TS_PAUSED:
    case TS_FINISHED:
    case TS_FAULT:
        break;
    }

    ESP_LOGI(TAG, "TS state -> %d", next);
}

/* ================= PUBLIC API ================= */

void thermal_shock_init(uint32_t max_cycles)
{
    ts_data.state       = TS_IDLE;
    ts_data.mode        = TS_MODE_HOT;   /* Initial entry point */
    ts_data.cycle_count = 0;
    ts_data.max_cycles  = max_cycles;
    ts_data.init_done   = true;
    
    thermal_shock_reset();

    ESP_LOGI(TAG,
             "Thermal shock initialized (max_cycles=%lu)",
             max_cycles);
}

void thermal_shock_start(void)
{
    if (!ts_data.init_done) {
        ESP_LOGW(TAG, "START rejected: system not armed");
        return;
    }

    if (ts_data.state == TS_IDLE || ts_data.state == TS_FINISHED) {

        ts_data.cycle_count = 0; //1
        ts_data.mode = TS_MODE_HOT;

        sm_start();
        ts_enter_state(TS_RUNNING);

        ESP_LOGI(TAG, "Thermal shock STARTED");
        
        send_cycle();
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

    ts_data.mode = TS_MODE_HOT;
    ts_data.cycle_count = 0;

    ts_enter_state(TS_IDLE);

    ESP_LOGI(TAG, "Thermal shock RESET");
}

/* ================= CALLBACKS FROM FSM ================= */
/* These are called ONLY by state_machine.c */

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

/* Called ONLY after COLD dwell completes */
void thermal_shock_notify_cycle_complete(void)
{
    ts_data.cycle_count++;
    
	send_cycle();
    
    ESP_LOGI(TAG,
             "Cycle %lu / %lu completed",
             ts_data.cycle_count,
             ts_data.max_cycles);

    if (ts_data.cycle_count >= ts_data.max_cycles) {

        /* Stop FSM first (prevents new relay requests) */
        sm_stop();

        /* FORCE RELAYS OFF (single authoritative place) */
        actuator_send_command(
            CMD_FORCE_RELAY,
            0x0000,   /* HOT=0, COLD=0 */
            0
        );

        ts_enter_state(TS_FINISHED);

        ESP_LOGI(TAG, "Thermal shock FINISHED");
    }
}

ts_state_t thermal_shock_get_state(void)
{
    return ts_data.state;
}

ts_mode_t thermal_shock_get_mode(void)
{
    return ts_data.mode;
}

uint32_t thermal_shock_get_cycle_count(void)
{
    return ts_data.cycle_count;
}

uint32_t thermal_shock_get_max_cycles(void)
{
    return ts_data.max_cycles;
}
