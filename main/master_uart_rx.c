#include "crc16.h"
#include "protocol.h"
#include "master_link.h"
#include "board_pins.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#define RX_BUF_SIZE       256
#define MAX_PAYLOAD_SIZE   64

#include "protocol.h"
#include "master_command.h"

static void handle_packet(const packet_header_t *hdr, const uint8_t *payload)
{
    if (hdr->type == PKT_COMMAND) {
        const payload_command_t *cmd =
            (const payload_command_t *)payload;

        master_handle_command(cmd);
    }
}


void master_uart_rx_task(void *arg)
{
    uint8_t rx_buf[RX_BUF_SIZE];
    size_t  rx_len = 0;

    while (1) {
        int n = uart_read_bytes(UART_UI, rx_buf + rx_len,
                                RX_BUF_SIZE - rx_len, pdMS_TO_TICKS(100));

        if (n <= 0) continue;
        rx_len += n;

        if (rx_len >= RX_BUF_SIZE) {
            ESP_LOGW("MASTER_UART_RX", "RX buffer overflow, clearing");
            rx_len = 0;
            continue;
        }

        /* Try to extract packets */
        while (rx_len >= sizeof(packet_header_t) + 2) { // at least header + CRC
            packet_header_t hdr;
            memcpy(&hdr, rx_buf, sizeof(hdr));

            if (hdr.magic != PACKET_MAGIC || hdr.version != PROTOCOL_VERSION) {
                memmove(rx_buf, rx_buf + 1, --rx_len);
                continue;
            }

            size_t total_len = sizeof(packet_header_t) + hdr.length + 2; // include CRC
            if (rx_len < total_len) break;

            /* Verify CRC */
            uint16_t crc_recv = (rx_buf[total_len - 2] << 8) | rx_buf[total_len - 1];
            uint16_t crc_calc = crc16_ccitt(rx_buf, total_len - 2, 0);

            if (crc_calc != crc_recv) {
                ESP_LOGW("MASTER_UART_RX",
                         "CRC fail seq=%lu type=%02X calc=%04X recv=%04X",
                         hdr.sequence, hdr.type, crc_calc, crc_recv);

                memmove(rx_buf, rx_buf + 1, --rx_len); // drop 1 byte
                continue;
            }

			/* Valid packet */
			const uint8_t *payload = rx_buf + sizeof(packet_header_t);
			
			switch (hdr.type) {
			
			case PKT_ACK: {
			    payload_ack_t ack;
			    memcpy(&ack, payload, sizeof(ack));
			
			    master_link_handle_ack(&hdr, &ack);
			
			    ESP_LOGI("MASTER_UART_RX",
			             "ACK RX cmd=%u status=%u seq=%lu",
			             ack.cmd_id, ack.status, hdr.sequence);
			    break;
			}
			
			case PKT_COMMAND:
			    handle_packet(&hdr, payload);
			    break;
			
			default:
			    ESP_LOGW("MASTER_UART_RX",
			             "Unhandled packet type: 0x%02X", hdr.type);
			    break;
			}


            /* Consume packet */
            memmove(rx_buf, rx_buf + total_len, rx_len - total_len);
            rx_len -= total_len;
        }
    }
}
