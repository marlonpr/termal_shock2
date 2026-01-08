#include "app_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "ds3231.h"
#include "lora.h"
#include "bus.h"
#include "board_pins.h"

static ds3231_dev_t rtc;
static lora_dev_t    lora;


#include "esp_log.h"

static const char *TAG = "APP_MASTER";

/* ================= TASK HANDLES ================= */
static TaskHandle_t task_sensors;
static TaskHandle_t task_router;
static TaskHandle_t task_lora;

/* ================= LOCAL INIT ================= */

static void init_gpio_inputs(void)
{
    gpio_config_t cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask =
            (1ULL << PIN_FLOAT_1) |
            (1ULL << PIN_FLOAT_2),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);
}

static void init_subsystems(void)
{
    ESP_LOGI(TAG, "Initializing buses");
    bus_i2c_init();
    bus_spi_init();

    /* Step 3 – device init will go here */
    ESP_LOGI(TAG, "Subsystems initialized");
}

/* ================= TASKS ================= */

static void task_sensor_loop(void *arg)
{
    while (1) {
        ESP_LOGI(TAG, "Reading sensors");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void task_router_loop(void *arg)
{
    while (1) {
        ESP_LOGI(TAG, "Routing data");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void task_lora_loop(void *arg)
{
    while (1) {
        ESP_LOGI(TAG, "LoRa handler");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ================= PUBLIC API ================= */

void app_master_init(void)
{
    ESP_LOGI(TAG, "Master init started");

    /* ================= STEP 2: Buses (already validated) ================= */
    ESP_ERROR_CHECK(bus_i2c_init());
    ESP_ERROR_CHECK(bus_spi_init());

    /* ================= STEP 4A: DS3231 RTC ================= */
    ESP_ERROR_CHECK(
        ds3231_init(&rtc, i2c_bus)
    );

    ds3231_time_t initial_time = {
        .second = 0,
        .minute = 0,
        .hour = 12,
        .day_of_week = 1,
        .day = 1,
        .month = 1,
        .year = 2025
    };

    /* Optional: set only once during commissioning */
    // ds3231_set_time(&rtc, &initial_time);

    ESP_LOGI(TAG, "DS3231 initialized");

    /* ================= STEP 4B: LoRa RA-02 ================= */
    ESP_ERROR_CHECK(
        lora_init(&lora, spi_lora, PIN_LORA_RST)
    );

    lora_set_frequency(&lora, 915000000);   // adjust if needed
    lora_enable_rx(&lora);

    ESP_LOGI(TAG, "LoRa initialized");

    /* ================= GPIO INPUTS ================= */
    init_gpio_inputs();

    ESP_LOGI(TAG, "Master init complete");
}

void app_master_start(void)
{
    ESP_LOGI(TAG, "Master start");

    xTaskCreate(task_sensor_loop, "sensor_task", 4096, NULL, 5, &task_sensors);
    xTaskCreate(task_router_loop, "router_task", 4096, NULL, 4, &task_router);
    xTaskCreate(task_lora_loop,   "lora_task",   4096, NULL, 3, &task_lora);
}
