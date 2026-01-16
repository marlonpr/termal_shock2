#pragma once
#include <stdint.h>
#include <stdbool.h>




#include "master_transport.h"
#include "protocol.h"

#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

/* Configuration */
#define ACK_TIMEOUT_MS   500
#define ACK_MAX_RETRIES  3

static const char *TAG = "MASTER_LINK";


typedef void (*ack_callback_t)(
    uint8_t cmd_id,
    uint8_t status,
    bool    success
);



/* =========================================================
 * Pending ACK command state
 * ========================================================= */

typedef struct {
    bool            active;
    uint8_t         cmd_id;
    uint32_t        sequence;
    uint32_t        deadline_ms;
    uint8_t         retries;
    ack_callback_t  callback;

    uint8_t         packet[64];
    size_t          packet_len;
} pending_cmd_t;

static pending_cmd_t pending;
static uint32_t sequence_counter = 1;

/* ========================================================= */














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


bool master_link_send_command_noack(
    uint8_t  cmd_id,
    uint16_t param16,
    uint32_t param32
);

bool master_link_send_command_noack_2(
    uint8_t  cmd_id,
    uint16_t param16,
    uint32_t param32
);


void master_link_send_bytes(const uint8_t *data, size_t len);
