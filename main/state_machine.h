#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SM_IDLE,
    SM_PREHEAT,
    SM_HOT_DWELL,
    SM_COLD_DWELL,
    SM_WAIT
} sm_state_t;
/* ================= CONTEXT ================= */

typedef struct {
    sm_state_t state;
    uint32_t   state_enter_sec;

    bool       relay_hot;
    bool       relay_cold;

    uint8_t    event_sent_mask;   // per-table bitmask
} sm_ctx_t;

extern sm_ctx_t ctx;


/* ================= API ================= */
void sm_init(void);
void sm_start(void);
void sm_stop(void);
void sm_update_temperatures(float t1, float t2, float t3);
void sm_update_floats(bool f1, bool f2);
void sm_tick(void);
bool sm_relay_hot_on(void);
bool sm_relay_cold_on(void);
sm_state_t sm_get_state(void);

/* --- NEW: Elapsed seconds since state entered --- */
uint32_t sm_elapsed_sec(void);

#endif /* STATE_MACHINE_H */
