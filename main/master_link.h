#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "protocol.h"

/* Configuration */
#define ACK_TIMEOUT_MS   500
#define ACK_MAX_RETRIES  3

typedef void (*ack_callback_t)(
    uint8_t cmd_id,
    uint8_t status,
    bool    success
);

/* API */
void master_link_init(void);

bool master_link_send_command(
    uint8_t  cmd_id,
    uint16_t param16,
    uint32_t param32,
    ack_callback_t cb
);

/* Called from RX task when ACK arrives */
void master_link_handle_ack(
    const packet_header_t *hdr,
    const payload_ack_t   *ack
);

/* Called periodically (timer or task) */
void master_link_tick(uint32_t now_ms);
