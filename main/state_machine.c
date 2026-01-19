// state_machine.c

#include "state_machine.h"
#include "termal_shock.h"
#include "rtc_service.h"
#include "esp_log.h"
#include "master_link.h"
#include <string.h>

#define TAG "STATE_MACHINE"

/* ================= RELAY DEFINITIONS ================= */

/*
 * Relay bit mapping (logical relays, not GPIOs)
 *
 * bit 0..3 : HOT heaters
 * bit 4    : COMMON pump
 * bit 5    : COMMON fan
 */

#define RELAY_HOT_HEATERS   0x0F  // bits 0..3
#define RELAY_COMMON_PUMP  (1 << 4)
#define RELAY_COMMON_FAN   (1 << 5)

#define RELAY_COMMON_ALL   (RELAY_COMMON_PUMP | RELAY_COMMON_FAN)

/* ================= DWELL EVENTS ================= */

typedef struct {
    uint32_t time_sec;
    uint8_t  relay_mask;   // FINAL desired relay state
} relay_event_t;

/* HOT dwell sequence */
static const relay_event_t hot_dwell_events[] = 
{
   // {   0, RELAY_COMMON_PUMP | RELAY_HOT_HEATERS },
	{   0,0x22},
	{   90,0x02},
};

/* COLD dwell sequence */
static const relay_event_t cold_dwell_events[] = 
{
   // {   0, RELAY_COMMON_PUMP | RELAY_COMMON_FAN },
	{   0,0x01},
	{   90,0x02},
};

#define HOT_EVENT_COUNT  \
    (sizeof(hot_dwell_events) / sizeof(hot_dwell_events[0]))

#define COLD_EVENT_COUNT \
    (sizeof(cold_dwell_events) / sizeof(cold_dwell_events[0]))

_Static_assert(HOT_EVENT_COUNT  <= 16, "Too many HOT dwell events");
_Static_assert(COLD_EVENT_COUNT <= 16, "Too many COLD dwell events");

/* ================= CONFIG ================= */

#define HOT_DWELL_SEC   90
#define COLD_DWELL_SEC  90
#define WAIT_SEC        30

#define CMD_SET_RELAYS  0x04




/* ================= ACTUATOR ================= */

static void actuator_set_relays(uint8_t relay_mask)
{
    master_link_send_command_noack(
        CMD_SET_RELAYS,
        relay_mask,
        0
    );

    ESP_LOGI(TAG, "TX RELAYS MASK=0x%02X", relay_mask);
}

/* ================= TIME HELPERS ================= */

static uint32_t now_sec(void)
{
    return rtc_get_epoch_seconds();
}

static uint32_t elapsed_sec(void)
{
    return now_sec() - ctx.state_enter_sec;
}

/* ================= DWELL OUTPUT HANDLER ================= */

static void sm_handle_dwell_outputs(void)
{
    const relay_event_t *table = NULL;
    uint8_t event_count = 0;

    if (ctx.state == SM_HOT_DWELL) {
        table = hot_dwell_events;
        event_count = HOT_EVENT_COUNT;
    }
    else if (ctx.state == SM_COLD_DWELL) {
        table = cold_dwell_events;
        event_count = COLD_EVENT_COUNT;
    }
    else {
        return;
    }

    uint32_t elapsed = elapsed_sec();

    for (int i = 0; i < event_count; i++) {

        if (ctx.event_sent_mask & (1 << i)) {
            continue;
        }

        if (elapsed < table[i].time_sec) {
            continue;
        }

        actuator_set_relays(table[i].relay_mask);
        ctx.event_sent_mask |= (1 << i);
    }
}

/* ================= STATE TRANSITIONS ================= */

static void sm_enter_state(sm_state_t next)
{
    ctx.state = next;
    ctx.state_enter_sec = now_sec();
    ctx.event_sent_mask = 0;

    ctx.relay_hot  = false;
    ctx.relay_cold = false;

    switch (next) {

    case SM_IDLE:
        actuator_set_relays(0x00);
        ESP_LOGI(TAG, "FSM -> IDLE");
        break;

    case SM_HOT_DWELL:
        ctx.relay_hot = true;
        thermal_shock_notify_hot();
        ESP_LOGI(TAG, "FSM -> HOT DWELL");
        break;

    case SM_COLD_DWELL:
        ctx.relay_cold = true;
        thermal_shock_notify_cold();
        ESP_LOGI(TAG, "FSM -> COLD DWELL");
        break;

    case SM_WAIT:
        //actuator_set_relays(0x00);
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
    ctx.state_enter_sec = 0;
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

    sm_handle_dwell_outputs();

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


/*
static void actuator_relay_ack_cb(uint8_t cmd,
                                  uint8_t status,
                                  bool ok)
{
    ESP_LOGI("ACTUATOR_CMD",
             "Relay ACK cmd=%u status=%u ok=%d",
             cmd, status, ok);
}

*/



/*

static const relay_event_t dwell_events[] = {
    // time, HOT-only, COLD-only, COMMON //

    // HOT dwell //
    {  0, RELAY_HOT_HEATERS, 0x00, RELAY_COMMON_ALL },
    { 3, 0x02,              0x00, RELAY_COMMON_ALL }, // drop one heater
    { 60, 0x03,              0x00, RELAY_COMMON_ALL }, // drop more

    // COLD dwell //
    {  0, 0x00, 0x2F, RELAY_COMMON_ALL },
    { 30, 0x00, 0x1F, RELAY_COMMON_ALL },
};



*/

/*


static const relay_event_t dwell_events[] = {
    // time, HOT mask, COLD mask //

    {   0, 0x01, 0x00 },   // HOT start
    {  3, 0x02, 0x00 },   // HOT @60s
    {  6, 0x03, 0x00 },   // HOT @61s
    {  9, 0x04, 0x00 },   // HOT @90s

    {   0, 0x00, 0x04 },   // COLD start
    {  3, 0x00, 0x03 },   // COLD @60s
    {  6, 0x00, 0x02 },   // COLD @61s
    {  9, 0x00, 0x01 },   // COLD @90s
};
*/