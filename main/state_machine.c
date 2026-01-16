// state_machine.c
#include "state_machine.h"
#include "termal_shock.h"
#include "rtc_service.h"
#include "esp_log.h"
#include <string.h>


#define TAG4 "STATE_MACHINE"

#include "master_link.h"


typedef struct {
    uint32_t time_sec;
    uint8_t  hot_mask;
    uint8_t  cold_mask;
} relay_event_t;



/* HOT-only relays */
#define RELAY_HOT_HEATERS   0x0F    // bits 0..3

/*
 * NOTE:
 * COMMON relays are physically wired on the HOT relay bank.
 * They are enabled during both HOT and COLD dwell by FSM logic.
 */


/* COMMON relays (still HOT pins physically) */
#define RELAY_COMMON_PUMP   (1 << 4)
#define RELAY_COMMON_FAN    (1 << 5)

#define RELAY_COMMON_ALL    (RELAY_COMMON_PUMP | RELAY_COMMON_FAN)


/* ================= SAFETY CHECKS ================= */

_Static_assert(
    (RELAY_COMMON_ALL & RELAY_HOT_HEATERS) == 0,
    "COMMON and HOT-only relays overlap"
);

static const relay_event_t dwell_events[] = {
    // time, HOT mask, COLD mask //

    {   0, 0x1C, 0x00 },   
   // {  5, 0x0E, 0x00 },   // HOT @60s
   // {  30, 0x3E, 0x00 },   // HOT @61s
    
    
        
    {  90, 0x10, 0x00 },   // COLD @61s    
        
    

    {   0, 0x00, 0x01 },   // COLD start
    {  90, 0x00, 0x0E },   // COLD @60s
    
    
    
    
    
    
    
    //{  95, 0x00, 0x0A },   // COLD @60s

    
    
    
        
        
        
        
        
        
        
        
        
        
        
        
    
};


#define DWELL_EVENT_COUNT \
    (sizeof(dwell_events) / sizeof(dwell_events[0]))
    
    _Static_assert(
    DWELL_EVENT_COUNT <= 16,
    "Too many dwell events for event_sent_mask"
);



/* ================= CONFIG ================= */
#define HOT_DWELL_SEC   90
#define COLD_DWELL_SEC  90
#define WAIT_SEC        30
#include "master_link.h"
#include "esp_log.h"

//static const char *TAG5 = "ACTUATOR_CMD";

/* CMD ID must be defined HERE, not in FSM */
#define CMD_SET_RELAYS  0x04



void actuator_set_relays(uint8_t hot_mask, uint8_t cold_mask)
{
    uint16_t p16 = ((uint16_t)cold_mask << 8) | hot_mask;

    master_link_send_command_noack(
            CMD_SET_RELAYS,
            p16,
            0) ;

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
    
    uint8_t    event_sent_mask;  // one bit per table entry

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



static void sm_handle_dwell_outputs(void)
{
    uint32_t elapsed = now_sec() - ctx.state_enter_sec;

    for (int i = 0; i < DWELL_EVENT_COUNT; i++) {

        /* Skip already sent events */
        if (ctx.event_sent_mask & (1 << i)) {
            continue;
        }

        /* Time not reached yet */
        if (elapsed < dwell_events[i].time_sec) {
            continue;
        }

        /* Enforce state */
        if (ctx.state == SM_HOT_DWELL &&
            dwell_events[i].hot_mask != 0) {

            actuator_set_relays(
                dwell_events[i].hot_mask,
                0x00
            );

            ctx.event_sent_mask |= (1 << i);
        }

        else if (ctx.state == SM_COLD_DWELL &&
                 dwell_events[i].cold_mask != 0) {

            actuator_set_relays(
                0x00,
                dwell_events[i].cold_mask
            );

            ctx.event_sent_mask |= (1 << i);
        }
        
        
        
        
        else if (ctx.state == SM_WAIT &&
                 dwell_events[i].cold_mask != 0) {

            actuator_set_relays(
                0x00,
                dwell_events[i].cold_mask
            );

            ctx.event_sent_mask |= (1 << i);
        }
        
        
        
    }
}

/* ================= STATE TRANSITION ================= */

static void sm_enter_state(sm_state_t next)
{
    ctx.state = next;
    ctx.event_sent_mask = 0;
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
        //actuator_set_relays(0x05, 0x00);
        ESP_LOGI(TAG, "FSM -> HOT");
        break;

    case SM_COLD_DWELL:
        ctx.relay_cold = true;
        thermal_shock_notify_cold();
        //actuator_set_relays(0x00, 0x0A);
        ESP_LOGI(TAG, "FSM -> COLD");
        break;

	case SM_WAIT:
	    //actuator_set_relays(0x00, 0x00);
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