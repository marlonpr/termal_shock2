#include "master_link.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "MASTER_LINK";

/* Transport hook (you already have this) */
extern void master_send_bytes(const uint8_t *data, size_t len);

/* Internal state */
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

/* ================================================= */

void master_link_init(void)
{
    memset(&pending, 0, sizeof(pending));
}

/* ================================================= */

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

    uint8_t buf[64];

    uint32_t seq = sequence_counter++;   // <-- FIX

    size_t len = protocol_build_command_packet(
        buf,
        sizeof(buf),
        seq,
        cmd_id,
        param16,
        param32
    );

    if (!len) {
        return false;
    }

    master_send_bytes(buf, len);

    pending.active      = true;
    pending.cmd_id      = cmd_id;
    pending.sequence    = seq;
    pending.retries     = 0;
    pending.callback    = cb;
    pending.deadline_ms =
        (esp_timer_get_time() / 1000) + ACK_TIMEOUT_MS;

    ESP_LOGI(TAG,
        "CMD %u sent seq=%lu",
        cmd_id,
        seq
    );

    return true;
}


/* ================================================= */

void master_link_handle_ack(
    const packet_header_t *hdr,
    const payload_ack_t   *ack
)
{
    if (!pending.active) return;
    if (hdr->sequence != pending.sequence) return;

    ESP_LOGI(TAG,
        "ACK received cmd=%u status=%u",
        ack->cmd_id, ack->status
    );

    if (pending.callback) {
        pending.callback(
            ack->cmd_id,
            ack->status,
            ack->status == ACK_OK
        );
    }

    pending.active = false;
}

/* ================================================= */

void master_link_tick(uint32_t now_ms)
{
    if (!pending.active) return;

    if (now_ms < pending.deadline_ms) return;

    if (pending.retries >= ACK_MAX_RETRIES) {
        ESP_LOGE(TAG, "CMD %u failed (timeout)", pending.cmd_id);

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
        pending.cmd_id
    );
}
