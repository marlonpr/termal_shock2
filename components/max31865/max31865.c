#include "max31865.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define REG_CONFIG      0x00
#define REG_RTD_MSB     0x01

#define CONFIG_BIAS     0x80
#define CONFIG_MODEAUTO 0x40
#define CONFIG_1SHOT    0x20
#define CONFIG_3WIRE    0x10
#define CONFIG_FAULTCLR 0x02

#define RREF 430.0f
#define RNOMINAL 100.0f

static void write_reg(spi_device_handle_t spi, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg | 0x80, val };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx
    };
    spi_device_transmit(spi, &t);
}

static uint16_t read_rtd(spi_device_handle_t spi)
{
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

esp_err_t max31865_init(
    max31865_t *dev,
    spi_device_handle_t spi,
    max31865_wire_t wire_mode
)
{
    if (!dev || !spi) return ESP_ERR_INVALID_ARG;

    dev->spi = spi;
    dev->wire_mode = wire_mode;

    uint8_t cfg = CONFIG_BIAS | CONFIG_MODEAUTO | CONFIG_FAULTCLR;
    if (wire_mode == MAX31865_3WIRE) {
        cfg |= CONFIG_3WIRE;
    }

    write_reg(spi, REG_CONFIG, cfg);
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

esp_err_t max31865_read_temperature(
    max31865_t *dev,
    float *temperature_c
)
{
    if (!dev || !temperature_c) return ESP_ERR_INVALID_ARG;

    uint16_t rtd = read_rtd(dev->spi);
    float resistance = (rtd * RREF) / 32768.0f;

    /* Callendar–Van Dusen (simplified for PT100) */
    float temp = (resistance - RNOMINAL) / 0.385f;

    *temperature_c = temp;
    return ESP_OK;
}
