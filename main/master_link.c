#include "master_link.h"

#include "protocol.h"

#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>




#include "master_transport.h"
#include "protocol.h"
#include "board_pins.h"

#include "driver/uart.h"
#include "esp_log.h"

#include "crc16.h"


/* Transport hook (provided elsewhere) */
extern void master_send_bytes(const uint8_t *data, size_t len);



void master_link_init(void)
{
    memset(&pending, 0, sizeof(pending));
    sequence_counter = 1;
}

/* =========================================================
 * ACK-REQUIRED COMMAND
 * ========================================================= */

bool master_link_send_command(
    uint8_t  cmd_id,
    uint16_t param16,
    uint32_t param32,
    ack_callback_t cb
)
{
    if (pending.active) {
        ESP_LOGW(TAG, "Command already pending");
        return false;
    }

    uint32_t seq = sequence_counter++;

    size_t len = protocol_build_command_packet(
        pending.packet,
        sizeof(pending.packet),
        seq,
        cmd_id,
        param16,
        param32
    );

    if (!len) {
        ESP_LOGE(TAG, "Failed to build command %u", cmd_id);
        return false;
    }

    pending.packet_len = len;

    master_send_bytes(pending.packet, pending.packet_len);

    pending.active      = true;
    pending.cmd_id      = cmd_id;
    pending.sequence    = seq;
    pending.retries     = 0;
    pending.callback    = cb;
    pending.deadline_ms =
        (esp_timer_get_time() / 1000) + ACK_TIMEOUT_MS;

    ESP_LOGI(TAG, "CMD %u sent (ACK) seq=%lu", cmd_id, seq);

    return true;
}

/* =========================================================
 * NO-ACK COMMAND (fire-and-forget)
 * ========================================================= */

bool master_link_send_command_noack(
    uint8_t  cmd_id,
    uint16_t param16,
    uint32_t param32
)
{
    uint8_t packet[64];

    uint32_t seq = sequence_counter++;

    size_t len = protocol_build_command_packet(
        packet,
        sizeof(packet),
        seq,
        cmd_id,
        param16,
        param32
    );

    if (!len) {
        ESP_LOGE(TAG, "Failed to build NO-ACK command %u", cmd_id);
        return false;
    }

    master_send_bytes(packet, len);

    ESP_LOGI(TAG, "CMD %u sent (NO-ACK) seq=%lu", cmd_id, seq);

    return true;
}








bool master_link_send_command_noack_2(
    uint8_t  cmd_id,
    uint16_t param16,
    uint32_t param32
)
{
    uint8_t packet[64];

    uint32_t seq = sequence_counter++;

    size_t len = protocol_build_command_packet(
        packet,
        sizeof(packet),
        seq,
        cmd_id,
        param16,
        param32
    );

    if (!len) {
        ESP_LOGE(TAG, "Failed to build NO-ACK command %u", cmd_id);
        return false;
    }

    master_send_bytes_2(packet, len);

    ESP_LOGI(TAG, "CMD %u sent (NO-ACK) seq=%lu", cmd_id, seq);

    return true;
}







/* =========================================================
 * ACK HANDLER
 * ========================================================= */

void master_link_handle_ack(
    const packet_header_t *hdr,
    const payload_ack_t   *ack
)
{
    if (!pending.active) {
        return;
    }

    if (hdr->sequence != pending.sequence) {
        return;
    }

    ESP_LOGI(TAG,
             "ACK RX cmd=%u status=%u",
             ack->cmd_id,
             ack->status);

    if (pending.callback) {
        pending.callback(
            ack->cmd_id,
            ack->status,
            ack->status == ACK_OK
        );
    }

    pending.active = false;
}

/* =========================================================
 * TIMEOUT / RETRY TICK
 * ========================================================= */

void master_link_tick(uint32_t now_ms)
{
    if (!pending.active) {
        return;
    }

    if (now_ms < pending.deadline_ms) {
        return;
    }

    if (pending.retries >= ACK_MAX_RETRIES) {

        ESP_LOGE(TAG,
                 "CMD %u failed (timeout)",
                 pending.cmd_id);

        if (pending.callback) {
            pending.callback(
                pending.cmd_id,
                ACK_ERR_UNKNOWN,
                false
            );
        }

        pending.active = false;
        return;
    }

    /* Retry */
    pending.retries++;

    master_send_bytes(
        pending.packet,
        pending.packet_len
    );

    pending.deadline_ms = now_ms + ACK_TIMEOUT_MS;

    ESP_LOGW(TAG,
             "Retry %u for CMD %u",
             pending.retries,
             pending.cmd_id);
}


void master_link_send_bytes(const uint8_t *data, size_t len)
{
    uart_write_bytes(UART_UI, (const char *)data, len);
}

