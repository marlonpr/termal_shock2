#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define TAG "MAX31865_TEST"

/* ===== SPI PINS ===== */
#define PIN_SCK     14  //CLK
#define PIN_MOSI    13	//SDI
#define PIN_MISO    12	//SDO
#define PIN_CS      16	//CS

/* ===== MAX31865 REGISTERS ===== */
#define REG_CONFIG      0x00
#define REG_RTD_MSB     0x01

#define CONFIG_BIAS     0x80
#define CONFIG_AUTO     0x40
#define CONFIG_3WIRE    0x10
#define CONFIG_CLEAR    0x02
#define CONFIG_FILT50   0x01

static spi_device_handle_t spi;

/* ===== SPI LOW-LEVEL ===== */
static uint8_t max_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { reg & 0x7F, 0x00 };
    uint8_t rx[2] = { 0 };

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    spi_device_transmit(spi, &t);
    return rx[1];
}

static void max_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg | 0x80, val };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx
    };

    spi_device_transmit(spi, &t);
}

/* ===== MAX31865 INIT ===== */
static void max31865_init(void)
{
    uint8_t config =
        CONFIG_BIAS |
        CONFIG_AUTO |
        CONFIG_3WIRE |
        CONFIG_CLEAR |
        CONFIG_FILT50;

    max_write_reg(REG_CONFIG, config);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/* ===== TEMPERATURE READ ===== */
static float max31865_read_temp(void)
{
    uint16_t rtd =
        ((uint16_t)max_read_reg(REG_RTD_MSB) << 8) |
        max_read_reg(REG_RTD_MSB + 1);

    rtd >>= 1;

    float ref_resistor = 430.0f;
    float resistance = (rtd * ref_resistor) / 32768.0f;

    /* Callendar–Van Dusen (PT100) */
    float Z1 = -242.02;
    float Z2 = 2.2228;
    float Z3 = 2.5859e-3;
    float Z4 = -4.8260e-6;

    return Z1 +
           Z2 * resistance +
           Z3 * resistance * resistance +
           Z4 * resistance * resistance * resistance;
}

/* ===== MAIN ===== */
void app_main(void)
{
    ESP_LOGI(TAG, "Initializing SPI...");

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    ESP_ERROR_CHECK(
        spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED)
    );

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 1,
        .spics_io_num = PIN_CS,
        .queue_size = 1
    };

    ESP_ERROR_CHECK(
        spi_bus_add_device(SPI2_HOST, &devcfg, &spi)
    );

    ESP_LOGI(TAG, "Initializing MAX31865...");
    max31865_init();

    while (1) {
        float temp = max31865_read_temp();
        ESP_LOGI(TAG, "PT100 Temperature: %.2f °C", temp);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}












// ============================ Float level sensor ===========================
/*

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define FLOAT_SENSOR_GPIO GPIO_NUM_23

static const char *TAG = "FloatSensor";

void app_main(void)
{
    // Configure input pin with internal pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FLOAT_SENSOR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // enable internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Float sensor monitor started");

    while (1) {
        int level = gpio_get_level(FLOAT_SENSOR_GPIO);

        if (level == 0) {
            ESP_LOGI(TAG, "Float switch CLOSED (liquid reached level)");
        } else {
            ESP_LOGI(TAG, "Float switch OPEN (below level)");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}







*/







/*
#include "esp_log.h"
#include "app_master.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "System boot");

    app_master_init();
    app_master_start();
}

*/