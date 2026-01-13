#include "max31865.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "MAX31865"

// Registers and config bits
#define REG_CONFIG     0x00
#define REG_RTD_MSB    0x01
#define CONFIG_BIAS    0x80
#define CONFIG_AUTO    0x40
#define CONFIG_3WIRE   0x10
#define CONFIG_CLEAR   0x02

#define REF_RESISTOR 430.0f
#define RNOMINAL     100.0f

static void write_reg(spi_device_handle_t spi, uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { reg | 0x80, val };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx
    };
    spi_device_transmit(spi, &t);
}

static uint16_t read_rtd(spi_device_handle_t spi) {
    uint8_t tx[3] = { REG_RTD_MSB & 0x7F, 0x00, 0x00 };
    uint8_t rx[3] = { 0 };
    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    spi_device_transmit(spi, &t);
    return ((rx[1] << 8) | rx[2]) >> 1;
}

esp_err_t max31865_init(max31865_t *dev, spi_host_device_t spi_bus, int cs_pin, max31865_wire_t wire_mode) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    dev->wire_mode = wire_mode;
    dev->cs_pin = cs_pin;

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 1,
        .spics_io_num = cs_pin,
        .queue_size = 1
    };
    esp_err_t ret = spi_bus_add_device(spi_bus, &devcfg, &dev->spi);
    if (ret != ESP_OK) return ret;

    uint8_t cfg = CONFIG_BIAS | CONFIG_AUTO | CONFIG_CLEAR;
    if (wire_mode == MAX31865_3WIRE) cfg |= CONFIG_3WIRE;

    write_reg(dev->spi, REG_CONFIG, cfg);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "MAX31865 initialized (CS=%d)", cs_pin);
    return ESP_OK;
}

esp_err_t max31865_read_temperature(max31865_t *dev, float *temperature_c) {
    if (!dev || !temperature_c) return ESP_ERR_INVALID_ARG;
    uint16_t rtd = read_rtd(dev->spi);
    float resistance = (rtd * REF_RESISTOR) / 32768.0f;
    *temperature_c = (resistance - RNOMINAL) / 0.385f;
    return ESP_OK;
}
