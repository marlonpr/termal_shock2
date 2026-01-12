#include "app_master.h"
#include "master_link.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_log.h"

#include "bus.h"
#include "app_data.h"
#include "protocol.h"
#include "ds3231.h"
#include "lora.h"
#include "max31865.h"
#include "board_pins.h"

#include "termal_shock.h"

#include "state_machine.h"
#include "command_manager.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "master_transport.h"

#include "master_uart_rx.h"

#include "rtc_service.h"





/* Transport hook */
extern void master_send_bytes(const uint8_t *data, size_t len);

/* ================= LOG ================= */

static const char *TAG = "APP_MASTER";

/* ================= DEVICES ================= */

static ds3231_dev_t rtc;
static lora_dev_t   lora;

static max31865_t pt100_1;
static max31865_t pt100_2;
static max31865_t pt100_3;

static spi_device_handle_t spi_max1;
static spi_device_handle_t spi_max2;
static spi_device_handle_t spi_max3;


/* SPI handle for LoRa */
static spi_device_handle_t spi_lora;

/* ================= TASK HANDLES ================= */

static TaskHandle_t task_sensors;
static TaskHandle_t task_router;
//static TaskHandle_t task_lora;
static TaskHandle_t task_state_machine;
static TaskHandle_t task_master_link;

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

/* Create SPI device for LoRa (VSPI) */
static esp_err_t init_lora_spi_device(void)
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,   // 8 MHz (safe for SX1278)
        .mode = 0,
        .spics_io_num = PIN_LORA_CS,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX
    };

    return spi_bus_add_device(VSPI_HOST, &devcfg, &spi_lora);
}




static void init_max31865_devices(void)
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,   // 1 MHz (safe for MAX31865)
        .mode = 1,                           // SPI MODE 1 (IMPORTANT)
        .queue_size = 1
    };

    /* PT100 #1 */
    devcfg.spics_io_num = PIN_MAX_CS1;
    ESP_ERROR_CHECK(
        spi_bus_add_device(SPI_HOST_USED, &devcfg, &spi_max1)
    );
    ESP_ERROR_CHECK(
        max31865_init(&pt100_1, spi_max1, MAX31865_3WIRE)
    );

    /* PT100 #2 */
    devcfg.spics_io_num = PIN_MAX_CS2;
    ESP_ERROR_CHECK(
        spi_bus_add_device(SPI_HOST_USED, &devcfg, &spi_max2)
    );
    ESP_ERROR_CHECK(
        max31865_init(&pt100_2, spi_max2, MAX31865_3WIRE)
    );

    /* PT100 #3 */
    devcfg.spics_io_num = PIN_MAX_CS3;
    ESP_ERROR_CHECK(
        spi_bus_add_device(SPI_HOST_USED, &devcfg, &spi_max3)
    );
    ESP_ERROR_CHECK(
        max31865_init(&pt100_3, spi_max3, MAX31865_3WIRE)
    );

    ESP_LOGI(TAG, "MAX31865 PT100 sensors initialized");
}








/* ================= TASKS ================= */

static void task_sensor_loop(void *arg)
{
    ds3231_time_t now;

    while (1) {

        /* ===== RTC ===== */
        ds3231_get_time(&rtc, &now);

        g_system_data.year   = now.year;
        g_system_data.month  = now.month;
        g_system_data.day    = now.day;
        g_system_data.hour   = now.hour;
        g_system_data.minute = now.minute;
        g_system_data.second = now.second;

        /* ===== PT100 ===== */
        max31865_read_temperature(&pt100_1, &g_system_data.pt100_1);
        max31865_read_temperature(&pt100_2, &g_system_data.pt100_2);
        max31865_read_temperature(&pt100_3, &g_system_data.pt100_3);

        /* ===== FLOAT SWITCHES ===== */
        g_system_data.float_1 = !gpio_get_level(PIN_FLOAT_1);
        g_system_data.float_2 = !gpio_get_level(PIN_FLOAT_2);

        /* ===== SEQUENCE ===== */
        g_system_data.sequence++;

        ESP_LOGI(TAG,
            "SEQ=%lu | %04d-%02d-%02d %02d:%02d:%02d | "
            "T1=%.2f T2=%.2f T3=%.2f | F1=%d F2=%d",
            g_system_data.sequence,
            g_system_data.year,
            g_system_data.month,
            g_system_data.day,
            g_system_data.hour,
            g_system_data.minute,
            g_system_data.second,
            g_system_data.pt100_1,
            g_system_data.pt100_2,
            g_system_data.pt100_3,
            g_system_data.float_1,
            g_system_data.float_2
        );

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



static void task_router_loop(void *arg)
{
    uint32_t last_status_ms = 0;

    while (1) {
        uint32_t now_ms = esp_timer_get_time() / 1000;

        /* ================= ACK timeout + retry ================= */
        master_link_tick(now_ms);

        /* ================= Periodic STATUS ================= */
        if ((now_ms - last_status_ms) >= 500) {
            uint8_t pkt[64];
            size_t len = protocol_build_status_packet(pkt, sizeof(pkt));

            if (len > 0) {
                master_send_bytes(pkt, len);

                ESP_LOG_BUFFER_HEXDUMP("STATUS_TX_HEX", pkt, len, ESP_LOG_INFO);
            }

            last_status_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}





static void task_state_machine_loop(void *arg)
{
    sm_init();
    cmdmgr_init();

    while (1) {
        uint32_t now_ms = esp_timer_get_time() / 1000;

        sm_tick();
        cmdmgr_tick(now_ms);

        bool hot  = sm_relay_hot_on();
        bool cold = sm_relay_cold_on();

        static bool prev_hot  = false;
        static bool prev_cold = false;

        if (hot != prev_hot || cold != prev_cold) {
            cmdmgr_send_relay(hot, cold);
            prev_hot  = hot;
            prev_cold = cold;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


static void task_master_link_loop(void *arg)
{
    while (1) {
        uint32_t now_ms = esp_timer_get_time() / 1000;
        master_link_tick(now_ms);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void task_fsm_loop(void *arg)
{
    sm_init();
    sm_start();

    while (1) {
        // Update FSM inputs
        sm_update_temperatures(
            g_system_data.pt100_1,
            g_system_data.pt100_2,
            g_system_data.pt100_3
        );

        sm_update_floats(
            g_system_data.float_1,
            g_system_data.float_2
        );

        // Tick FSM
        sm_tick();

        // Send elapsed seconds to UI
        uint32_t elapsed = sm_elapsed_sec(); // or sm_state_elapsed_seconds()

        uint8_t buf[4];
        buf[0] = (elapsed >> 24) & 0xFF;
        buf[1] = (elapsed >> 16) & 0xFF;
        buf[2] = (elapsed >> 8) & 0xFF;
        buf[3] = elapsed & 0xFF;

        master_send_bytes(buf, sizeof(buf));

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz
    }
}






/* ================= PUBLIC API ================= */

void app_master_init(void)
{
    ESP_LOGI(TAG, "Master init started");

    /* ================= STEP 2: BUSES ================= */
    ESP_ERROR_CHECK(bus_i2c_init());
    ESP_ERROR_CHECK(bus_spi_init());


	/* ================= STEP 4C: MAX31865 ================= */
	init_max31865_devices();


    /* ================= STEP 3A: DS3231 RTC ================= */
    ESP_ERROR_CHECK(ds3231_init(&rtc, i2c_bus));
    ESP_LOGI(TAG, "DS3231 initialized");

    /* ================= STEP 3B: LoRa SPI DEVICE ================= */
    ESP_ERROR_CHECK(init_lora_spi_device());

    /* ================= STEP 3C: LoRa RA-02 ================= */
    ESP_ERROR_CHECK(lora_init(&lora, spi_lora, PIN_LORA_RST));
    lora_set_frequency(&lora, 915000000);   // change if needed
    lora_enable_rx(&lora);
    ESP_LOGI(TAG, "LoRa initialized");

    /* ================= GPIO INPUTS ================= */
    init_gpio_inputs();

	master_link_init();

	ESP_LOGI(TAG, "Initializing master UART transport");
	master_transport_init();

	thermal_shock_init(10);   // example: 10 cycles


    ESP_LOGI(TAG, "Master init complete");


}

void app_master_start(void)
{
    ESP_LOGI(TAG, "Master start");

    xTaskCreate(task_sensor_loop, "sensor_task", 4096, NULL, 5, &task_sensors);
    xTaskCreate(task_router_loop, "router_task", 4096, NULL, 4, &task_router);
    xTaskCreate(task_state_machine_loop,"state_machine",4096,NULL,6,&task_state_machine);
    xTaskCreate(task_master_link_loop, "master_link",   4096, NULL, 3, &task_master_link);
	xTaskCreate(master_uart_rx_task,    "uart_rx",4096,NULL,6,    NULL);
	xTaskCreate(task_thermal_shock, "ts_tick", 2048, NULL, 5, NULL);

   // FSM + UI publishing
    xTaskCreate(task_fsm_loop, "fsm_task", 4096, NULL, 6, NULL);

}




    //xTaskCreate(task_lora_loop,   "lora_task",   4096, NULL, 3, &task_lora);

/*
static void task_lora_loop(void *arg)
{
    while (1) {
        ESP_LOGI(TAG, "LoRa handler");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
*/
