// state_machine.c
#include "state_machine.h"
#include "termal_shock.h"
#include "rtc_service.h"   // rtc_get_epoch_seconds()
#include "esp_log.h"

#define TAG "STATE_MACHINE"

/* ================= CONFIG ================= */
#define HOT_TARGET_C  65.0f
#define COLD_TARGET_C 10.0f

#define HOT_DWELL_SEC   90
#define COLD_DWELL_SEC  90
#define WAIT_SEC        30

/* ================= CONTEXT ================= */
typedef struct {
    sm_state_t state;
    uint32_t   state_enter_sec;  // epoch seconds when entered

    float t1, t2, t3;
    bool  float1, float2;

    bool relay_hot;
    bool relay_cold;
} sm_ctx_t;

static sm_ctx_t ctx;

/* ================= HELPERS ================= */
static void sm_enter_state(sm_state_t next)
{
    ctx.state = next;
    ctx.state_enter_sec = rtc_get_epoch_seconds();
    ctx.relay_hot  = false;
    ctx.relay_cold = false;

    switch (next) {
        case SM_IDLE: break;
        case SM_PREHEAT: ctx.relay_hot = true; break;
        case SM_HOT_DWELL: ctx.relay_hot = true; thermal_shock_notify_hot(); break;
        case SM_COLD_DWELL: ctx.relay_cold = true; thermal_shock_notify_cold(); break;
        case SM_WAIT: break;
    }

    ESP_LOGI(TAG, "Entered FSM state %d at %lu s", next, ctx.state_enter_sec);
}

static bool elapsed_sec(uint32_t duration)
{
    uint32_t now = rtc_get_epoch_seconds();
    return (now - ctx.state_enter_sec) >= duration;
}

/* ================= API ================= */
void sm_init(void)
{
    ctx = (sm_ctx_t){0};
    sm_enter_state(SM_IDLE);
}

void sm_start(void)
{
    sm_enter_state(SM_PREHEAT);
}

void sm_stop(void)
{
    sm_enter_state(SM_IDLE);
}

void sm_update_temperatures(float t1, float t2, float t3)
{
    ctx.t1 = t1;
    ctx.t2 = t2;
    ctx.t3 = t3;
}

void sm_update_floats(bool f1, bool f2)
{
    ctx.float1 = f1;
    ctx.float2 = f2;
}

/* ================= FSM TICK ================= */
void sm_tick(void)
{
    switch (ctx.state) {
        case SM_PREHEAT:
            if (ctx.t1 >= HOT_TARGET_C && ctx.float1) sm_enter_state(SM_HOT_DWELL);
            break;
        case SM_HOT_DWELL:
            if (elapsed_sec(HOT_DWELL_SEC)) sm_enter_state(SM_COLD_DWELL);
            break;
        case SM_COLD_DWELL:
            if (elapsed_sec(COLD_DWELL_SEC)) {
                thermal_shock_notify_cycle_complete();
                sm_enter_state(SM_WAIT);
            }
            break;
        case SM_WAIT:
            if (elapsed_sec(WAIT_SEC)) sm_enter_state(SM_PREHEAT);
            break;
        default: break;
    }
}

/* ================= QUERIES ================= */
bool sm_relay_hot_on(void)  { return ctx.relay_hot; }
bool sm_relay_cold_on(void) { return ctx.relay_cold; }
sm_state_t sm_get_state(void) { return ctx.state; }
uint32_t sm_elapsed_sec(void) { return rtc_get_epoch_seconds() - ctx.state_enter_sec; }
