// state_machine.c
#include "state_machine.h"
#include "termal_shock.h"
#include "rtc_service.h"
#include "esp_log.h"
#include <string.h>

#define TAG4 "STATE_MACHINE"

#include "master_link.h"


/* ================= CONFIG ================= */
#define HOT_DWELL_SEC   9
#define COLD_DWELL_SEC  9
#define WAIT_SEC        3
#include "master_link.h"
#include "esp_log.h"

static const char *TAG = "ACTUATOR_CMD";

/* CMD ID must be defined HERE, not in FSM */
#define CMD_SET_RELAYS  0x04

static void actuator_relay_ack_cb(uint8_t cmd,
                                  uint8_t status,
                                  bool ok)
{
    ESP_LOGI("ACTUATOR_CMD",
             "Relay ACK cmd=%u status=%u ok=%d",
             cmd, status, ok);
}


void actuator_set_relays(uint8_t hot_mask, uint8_t cold_mask)
{
    uint16_t p16 = ((uint16_t)cold_mask << 8) | hot_mask;

    if (!master_link_send_command(
            CMD_SET_RELAYS,
            p16,
            0,
            actuator_relay_ack_cb)) {

        ESP_LOGW(TAG, "Relay command rejected (link busy)");
        return;
    }

    ESP_LOGI(TAG,
             "TX RELAYS HOT=0x%X COLD=0x%X",
             hot_mask,
             cold_mask);
}



/* ================= CONTEXT ================= */

typedef struct {
    sm_state_t state;
    uint32_t   state_enter_sec;

    bool relay_hot;
    bool relay_cold;
} sm_ctx_t;

static sm_ctx_t ctx;

/* ================= TIME HELPERS ================= */

static uint32_t now_sec(void)
{
    return rtc_get_epoch_seconds();
}

static uint32_t elapsed_sec(void)
{
    return now_sec() - ctx.state_enter_sec;
}

/* ================= STATE TRANSITION ================= */

static void sm_enter_state(sm_state_t next)
{
    ctx.state = next;
    ctx.state_enter_sec = now_sec();

    ctx.relay_hot  = false;
    ctx.relay_cold = false;

    switch (next) {

    case SM_IDLE:
        actuator_set_relays(0x00, 0x00);
        ESP_LOGI(TAG, "FSM -> IDLE");
        break;

    case SM_HOT_DWELL:
        ctx.relay_hot = true;
        thermal_shock_notify_hot();
        actuator_set_relays(0x0F, 0x00);
        ESP_LOGI(TAG, "FSM -> HOT");
        break;

    case SM_COLD_DWELL:
        ctx.relay_cold = true;
        thermal_shock_notify_cold();
        actuator_set_relays(0x00, 0x0F);
        ESP_LOGI(TAG, "FSM -> COLD");
        break;

    case SM_WAIT:
        actuator_set_relays(0x00, 0x00);
        ESP_LOGI(TAG, "FSM -> WAIT");
        break;

    default:
        break;
    }
}

/* ================= API ================= */

void sm_init(void)
{
    memset(&ctx, 0, sizeof(ctx));
    sm_enter_state(SM_IDLE);
}

void sm_start(void)
{
    /* Always start with HOT dwell */
    sm_enter_state(SM_HOT_DWELL);
}

void sm_stop(void)
{
    sm_enter_state(SM_IDLE);
}

/* ================= FSM TICK ================= */
/* Call once per second */

void sm_tick(void)
{
    uint32_t e = elapsed_sec();

    switch (ctx.state) {

    case SM_HOT_DWELL:
        ESP_LOGI(TAG, "HOT  %lu / %d", e, HOT_DWELL_SEC);
        if (e >= HOT_DWELL_SEC) {
            sm_enter_state(SM_WAIT);
        }
        break;

    case SM_COLD_DWELL:
        ESP_LOGI(TAG, "COLD %lu / %d", e, COLD_DWELL_SEC);
        if (e >= COLD_DWELL_SEC) {
            thermal_shock_notify_cycle_complete();
            sm_enter_state(SM_WAIT);
        }
        break;

    case SM_WAIT:
        ESP_LOGI(TAG, "WAIT %lu / %d", e, WAIT_SEC);
        if (e >= WAIT_SEC) {

            /* Decide next dwell based on LAST mode */
            if (ts_data.mode == TS_MODE_HOT) {
                sm_enter_state(SM_COLD_DWELL);
            } else {
                sm_enter_state(SM_HOT_DWELL);
            }
        }
        break;

    default:
        break;
    }
}

/* ================= QUERIES ================= */

bool sm_relay_hot_on(void)
{
    return ctx.relay_hot;
}

bool sm_relay_cold_on(void)
{
    return ctx.relay_cold;
}

sm_state_t sm_get_state(void)
{
    return ctx.state;
}

uint32_t sm_elapsed_sec(void)
{
    return elapsed_sec();
}
