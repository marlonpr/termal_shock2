#include "master_transport.h"
#include "protocol.h"
#include "board_pins.h"

#include "driver/uart.h"
#include "esp_log.h"

#include "crc16.h"


static const char *TAG = "MASTER_UART";

void master_transport_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(
        uart_driver_install(
            UART_RELAY,
            2048,   // RX buffer
            0,      // TX buffer (blocking)
            0,
            NULL,
            0
        )
    );

    ESP_ERROR_CHECK(
        uart_param_config(UART_RELAY, &cfg)
    );

    ESP_ERROR_CHECK(
        uart_set_pin(
            UART_RELAY,
            PIN_UART_RELAY_TX,
            PIN_UART_RELAY_RX,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE
        )
    );

    ESP_LOGI(TAG,
        "Master UART initialized (UART=%d TX=%d RX=%d)",
        UART_RELAY,
        PIN_UART_RELAY_TX,
        PIN_UART_RELAY_RX
    );
}


void master_send_bytes(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return;

    /* Compute CRC */
    uint16_t crc = crc16_ccitt(data, len, 0);

    /* Create a temporary buffer to append CRC */
    uint8_t buf[256];
    if (len + 2 > sizeof(buf)) return;  // safety
    memcpy(buf, data, len);
    buf[len]     = crc >> 8;
    buf[len + 1] = crc & 0xFF;
    len += 2;

    uart_write_bytes(UART_RELAY, (const char *)buf, len);
}

