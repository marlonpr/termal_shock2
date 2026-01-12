#include "command_manager.h"
#include "protocol.h"
#include "esp_log.h"

#define TAG "CMDMGR"

#define CMD_TIMEOUT_MS   1000
#define CMD_MAX_RETRIES  3

static struct {
    bool     active;
    uint32_t seq;
    uint32_t sent_at;
    uint8_t  retries;
} cmd;

static uint32_t seq_counter = 1;

void cmdmgr_init(void)
{
    cmd.active = false;
}

bool cmdmgr_send_relay(bool hot_on, bool cold_on)
{
    if (cmd.active) {
        ESP_LOGW(TAG, "Command already pending");
        return false;
    }

    uint8_t buf[64];

    uint16_t mask = (hot_on ? 0x01 : 0) |
                    (cold_on ? 0x02 : 0);

    size_t len = protocol_build_command_packet(
        buf,
        sizeof(buf),
        seq_counter,
        CMD_FORCE_RELAY,
        mask,
        0
    );

    if (!len) {
        ESP_LOGE(TAG, "Packet build failed");
        return false;
    }

    /* Transport send will be added later */
    ESP_LOGI(TAG, "CMD SEND seq=%lu hot=%d cold=%d",
             seq_counter, hot_on, cold_on);

    cmd.active   = true;
    cmd.seq      = seq_counter;
    cmd.sent_at  = 0;        // filled in tick
    cmd.retries  = 0;

    seq_counter++;
    return true;
}

void cmdmgr_tick(uint32_t now_ms)
{
    if (!cmd.active) return;

    if (cmd.sent_at == 0) {
        cmd.sent_at = now_ms;
        return;
    }

    if ((now_ms - cmd.sent_at) < CMD_TIMEOUT_MS)
        return;

    if (cmd.retries >= CMD_MAX_RETRIES) {
        ESP_LOGE(TAG, "CMD FAILED seq=%lu", cmd.seq);
        cmd.active = false;
        return;
    }

    cmd.retries++;
    cmd.sent_at = now_ms;

    ESP_LOGW(TAG, "CMD RETRY %d seq=%lu",
             cmd.retries, cmd.seq);

    /* Re-send command (same sequence) */
    /* Transport hook goes here */
}

void cmdmgr_handle_ack(uint32_t sequence, uint8_t status)
{
    if (!cmd.active || sequence != cmd.seq)
        return;

    if (status == 0) {
        ESP_LOGI(TAG, "CMD ACK OK seq=%lu", sequence);
    } else {
        ESP_LOGE(TAG, "CMD ACK ERR seq=%lu status=%d",
                 sequence, status);
    }

    cmd.active = false;
}
