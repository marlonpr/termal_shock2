#include "lora.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

#define TAG "LORA"

/* SX1278 registers (unchanged) */
#define REG_FIFO              0x00
#define REG_OP_MODE           0x01
#define REG_FRF_MSB           0x06
#define REG_FRF_MID           0x07
#define REG_FRF_LSB           0x08
#define REG_PA_CONFIG         0x09
#define REG_FIFO_ADDR_PTR     0x0D
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_FIFO_RX_BASE_ADDR 0x0F
#define REG_IRQ_FLAGS         0x12
#define REG_RX_NB_BYTES       0x13
#define REG_PKT_SNR_VALUE     0x19
#define REG_PKT_RSSI_VALUE    0x1A
#define REG_MODEM_CONFIG_1    0x1D
#define REG_MODEM_CONFIG_2    0x1E
#define REG_PAYLOAD_LENGTH   0x22
#define REG_MODEM_CONFIG_3    0x26
#define REG_FIFO_RX_CURRENT  0x10
#define REG_VERSION           0x42

#define MODE_LONG_RANGE_MODE  0x80
#define MODE_SLEEP            0x00
#define MODE_STDBY            0x01
#define MODE_TX               0x03
#define MODE_RX_CONTINUOUS    0x05

/* Low-level SPI helpers */

static void lora_write_reg(lora_dev_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg | 0x80, val };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx
    };
    spi_device_transmit(dev->spi, &t);
}

static uint8_t lora_read_reg(lora_dev_t *dev, uint8_t reg)
{
    uint8_t tx[2] = { reg & 0x7F, 0x00 };
    uint8_t rx[2] = { 0 };

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    spi_device_transmit(dev->spi, &t);
    return rx[1];
}

static void lora_reset(lora_dev_t *dev)
{
    gpio_set_level(dev->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(dev->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/* Public API */

esp_err_t lora_init(
    lora_dev_t *dev,
    spi_device_handle_t spi,
    gpio_num_t rst_pin
)
{
    if (!dev || !spi) return ESP_ERR_INVALID_ARG;

    dev->spi = spi;
    dev->rst_pin = rst_pin;

    lora_reset(dev);

    uint8_t version = lora_read_reg(dev, REG_VERSION);
    if (version != 0x12) {
        ESP_LOGE(TAG, "SX1278 not found (0x%02X)", version);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "SX1278 detected");

    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));
    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    lora_write_reg(dev, REG_MODEM_CONFIG_1, 0x72);
    lora_write_reg(dev, REG_MODEM_CONFIG_2, 0x74);
    lora_write_reg(dev, REG_MODEM_CONFIG_3, 0x04);
    lora_write_reg(dev, REG_PA_CONFIG, 0x8F);

    lora_write_reg(dev, REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(dev, REG_FIFO_RX_BASE_ADDR, 0x00);
    lora_write_reg(dev, REG_FIFO_ADDR_PTR, 0x80);

    return ESP_OK;
}

void lora_set_frequency(lora_dev_t *dev, long freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    lora_write_reg(dev, REG_FRF_MSB, (frf >> 16) & 0xFF);
    lora_write_reg(dev, REG_FRF_MID, (frf >> 8) & 0xFF);
    lora_write_reg(dev, REG_FRF_LSB, frf & 0xFF);
}

void lora_send_packet(lora_dev_t *dev, const uint8_t *data, int len)
{
    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    lora_write_reg(dev, REG_FIFO_ADDR_PTR, 0x80);
    lora_write_reg(dev, REG_IRQ_FLAGS, 0xFF);

    for (int i = 0; i < len; i++) {
        lora_write_reg(dev, REG_FIFO, data[i]);
    }

    lora_write_reg(dev, REG_PAYLOAD_LENGTH, len);
    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    while (!(lora_read_reg(dev, REG_IRQ_FLAGS) & 0x08)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    lora_write_reg(dev, REG_IRQ_FLAGS, 0xFF);
}

void lora_enable_rx(lora_dev_t *dev)
{
    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

int lora_receive_packet(lora_dev_t *dev, uint8_t *buf, int maxlen)
{
    int len = lora_read_reg(dev, REG_RX_NB_BYTES);
    if (len > 0) {
        if (len > maxlen) len = maxlen;
        uint8_t addr = lora_read_reg(dev, REG_FIFO_RX_CURRENT);
        lora_write_reg(dev, REG_FIFO_ADDR_PTR, addr);
        for (int i = 0; i < len; i++) {
            buf[i] = lora_read_reg(dev, REG_FIFO);
        }
        lora_write_reg(dev, REG_IRQ_FLAGS, 0xFF);
    }
    return len;
}
