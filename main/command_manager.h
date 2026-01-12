#pragma once
#include <stdint.h>
#include <stdbool.h>

void cmdmgr_init(void);

/* Called by state machine */
bool cmdmgr_send_relay(bool hot_on, bool cold_on);

/* Called periodically (task or timer) */
void cmdmgr_tick(uint32_t now_ms);

/* Called when an ACK packet is received */
void cmdmgr_handle_ack(uint32_t sequence, uint8_t status);
