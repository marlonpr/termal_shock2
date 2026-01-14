// state_machine.c
#include "state_machine.h"
#include "termal_shock.h"
#include "rtc_service.h"
#include "esp_log.h"

#define TAG "STATE_MACHINE"

/* ================= CONFIG ================= */

#define HOT_DWELL_SEC   90
#define COLD_DWELL_SEC  90
#define WAIT_SEC        30

/* ================= CONTEXT ================= */

typedef struct {
    sm_state_t state;
    uint32_t   state_enter_sec;
    sm_state_t prev_dwell;   // remembers HOT or COLD before WAIT
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

/* ================= STATE ENTRY ================= */

static void sm_enter_state(sm_state_t next)
{
    ctx.state = next;
    ctx.state_enter_sec = now_sec();
    ctx.relay_hot  = false;
    ctx.relay_cold = false;

    switch (next) {

    case SM_IDLE:
        ESP_LOGI(TAG, "FSM -> IDLE");
        break;

    case SM_HOT_DWELL:
        ctx.relay_hot = true;
        ctx.prev_dwell = SM_HOT_DWELL;
        thermal_shock_notify_hot();
        ESP_LOGI(TAG, "FSM -> HOT (0/%d)", HOT_DWELL_SEC);
        break;

    case SM_COLD_DWELL:
        ctx.relay_cold = true;
        ctx.prev_dwell = SM_COLD_DWELL;
        thermal_shock_notify_cold();
        ESP_LOGI(TAG, "FSM -> COLD (0/%d)", COLD_DWELL_SEC);
        break;

    case SM_WAIT:
        ESP_LOGI(TAG, "FSM -> WAIT (0/%d)", WAIT_SEC);
        break;

    case SM_PREHEAT:
        /* not used in this timing-only version */
        break;
    }
}

/* ================= API ================= */

void sm_init(void)
{
    ctx = (sm_ctx_t){0};
    sm_enter_state(SM_IDLE);
}

void sm_start(void)
{
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
            sm_enter_state(SM_WAIT);
        }
        break;

    case SM_WAIT:
        ESP_LOGI(TAG, "WAIT %lu / %d", e, WAIT_SEC);
        if (e >= WAIT_SEC) {

            if (ctx.prev_dwell == SM_HOT_DWELL) {
                sm_enter_state(SM_COLD_DWELL);
            } else {
                /* COLD completed → cycle complete */
                thermal_shock_notify_cycle_complete();
                sm_enter_state(SM_HOT_DWELL);
            }
        }
        break;

    default:
        break;
    }
}

/* ================= QUERIES ================= */

bool sm_relay_hot_on(void)  { return ctx.relay_hot; }
bool sm_relay_cold_on(void) { return ctx.relay_cold; }
sm_state_t sm_get_state(void) { return ctx.state; }
uint32_t sm_elapsed_sec(void) { return elapsed_sec(); }
