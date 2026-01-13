#include "app_master.h"
#include "app_data.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bus.h"
#include "ds3231.h"
#include "max31865.h"
#include "board_pins.h"
#include "state_machine.h"
#include "command_manager.h"
#include "master_transport.h"
#include "master_link.h"
#include "master_uart_rx.h"
#include "protocol.h"
#include "rtc_service.h"
#include "termal_shock.h"


static bool prev_float_1 = false;

static void process_float_gate(void)
{
    bool float_ok = g_system_data.float_1;

    /* FLOAT became OK (OPEN → CLOSED) */
    if (float_ok && !prev_float_1) {

        ts_data.init_done = true;

        ESP_LOGI("FLOAT", "Float OK → system armed");

        if (ts_data.state == TS_IDLE) {
            ts_enter_state(TS_INIT);
        }
    }

    /* FLOAT lost (CLOSED → OPEN) */
    else if (!float_ok && prev_float_1) {

        ESP_LOGE("FLOAT", "Float LOST → emergency reset");

        ts_data.init_done = false;

        thermal_shock_reset();

        actuator_send_command(
            CMD_FORCE_RELAY,
            0x00,   /* all relays off */
            0
        );

        ts_enter_state(TS_FAULT);
    }

    prev_float_1 = float_ok;
}



/* ================= LOG ================= */
static const char *TAG = "APP_MASTER";

typedef struct {
    gpio_num_t gpio;
    const char *name;
    int last_state;
} float_sensor_t;

static float_sensor_t sensors[] = {
    {
        .gpio = PIN_FLOAT_1,
        .name = "Tank_High",
        .last_state = -1
    },
    {
        .gpio = PIN_FLOAT_2,
        .name = "Tank_Low",
        .last_state = -1
    }
};

#define SENSOR_COUNT (sizeof(sensors) / sizeof(sensors[0]))

static void float_sensors_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   // external 4.7k pull-ups
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    for (int i = 0; i < SENSOR_COUNT; i++) {
        io_conf.pin_bit_mask = (1ULL << sensors[i].gpio);
        gpio_config(&io_conf);
    }
}



int read_debounced(gpio_num_t gpio, int sample_count, int delay_ms)
{
    int state = gpio_get_level(gpio);

    for (int i = 0; i < sample_count; i++) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        if (gpio_get_level(gpio) != state) {
            return -1;
        }
    }
    return state;
}

/* ================= DEVICES ================= */

#define NUM_PT100 2
static max31865_t max31865_dev[NUM_PT100];
static const int max31865_cs_pins[NUM_PT100] = {
    PIN_MAX31865_CS1,
    PIN_MAX31865_CS2
};

/* ================= TASK HANDLES ================= */
static TaskHandle_t task_sensors;
static TaskHandle_t task_router;
static TaskHandle_t task_fsm;



/* ================= SPI + MAX31865 INIT ================= */
static esp_err_t init_max31865_devices(void)
{
    ESP_LOGI(TAG, "Initializing SPI bus for MAX31865...");

    // SPI bus config (VSPI / SPI2_HOST)
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MAX31865_MISO,
        .mosi_io_num = PIN_MAX31865_MOSI,
        .sclk_io_num = PIN_MAX31865_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }

    // Initialize all MAX31865 devices
    for (int i = 0; i < NUM_PT100; i++) {
        ret = max31865_init(&max31865_dev[i], SPI2_HOST, max31865_cs_pins[i], MAX31865_3WIRE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init MAX31865 device %d (CS=%d)", i, max31865_cs_pins[i]);
            return ret;
        }
        ESP_LOGI(TAG, "MAX31865 device %d initialized (CS=%d)", i, max31865_cs_pins[i]);
    }

    return ESP_OK;
}

/* ================= SENSOR TASK ================= */
static void task_sensor_loop(void *arg)
{
    ds3231_time_t last_time = {0};
    float last_temp[NUM_PT100] = {0};

    while (1) {
        // ===== RTC =====
        if (rtc_service_get_time(&last_time) != ESP_OK) {
            ESP_LOGW(TAG, "RTC read failed, using last known time");
        }

        g_system_data.year   = last_time.year;
        g_system_data.month  = last_time.month;
        g_system_data.day    = last_time.day;
        g_system_data.hour   = last_time.hour;
        g_system_data.minute = last_time.minute;
        g_system_data.second = last_time.second;
        

        // ===== PT100 =====
        for (int i = 0; i < NUM_PT100; i++) {
            float temp;
            if (max31865_read_temperature(&max31865_dev[i], &temp) == ESP_OK) {
                last_temp[i] = temp;
            } else {
                ESP_LOGW(TAG, "Failed to read PT100 sensor %d, using last valid reading", i);
            }
            g_system_data.pt100[i] = last_temp[i];
        }

		for (int i = 0; i < SENSOR_COUNT; i++) {
		            int state = read_debounced(sensors[i].gpio, 5, 10);
		
		            if (state >= 0 && state != sensors[i].last_state) {
		                sensors[i].last_state = state;
		
		                if (state == 0) 
		                {
		                    ESP_LOGI("FLOAT", "%s: CLOSED (level reached)", sensors[i].name);
		                } else 
		                {
		                    ESP_LOGI("FLOAT", "%s: OPEN (below level)", sensors[i].name);
		                }
		            }
		        }


        g_system_data.float_1 = !sensors[0].last_state;
        g_system_data.float_2 = !sensors[1].last_state;
        
        
        process_float_gate();

        
        
              // ===== FLOAT SWITCHES =====
        //g_system_data.float_1 = !gpio_get_level(PIN_FLOAT_1);
        //g_system_data.float_2 = !gpio_get_level(PIN_FLOAT_2);
          
        
        
        
    
        
        
        
        
        
        

        // ===== SEQUENCE =====
        g_system_data.sequence++;

        ESP_LOGI(TAG,
            "SEQ=%lu |%02d | "
            "T1=%.2f T2=%.2f | F1=%d F2=%d",
            // %04d-%02d-%02d %02d:%02d:
            g_system_data.sequence,
            //g_system_data.year,
            //g_system_data.month,
            //g_system_data.day,
            //g_system_data.hour,
            //g_system_data.minute,
            g_system_data.second,
            g_system_data.pt100[0],
            g_system_data.pt100[1],
            g_system_data.float_1,
            g_system_data.float_2
        );

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ================= ROUTER TASK ================= */
static void task_router_loop(void *arg)
{
    uint32_t last_status_ms = 0;

    while (1) {
        uint32_t now_ms = esp_timer_get_time() / 1000;

        master_link_tick(now_ms);

        if ((now_ms - last_status_ms) >= 500) {
            uint8_t pkt[64];
            size_t len = protocol_build_status_packet(pkt, sizeof(pkt));

            if (len > 0) {
                master_send_bytes(pkt, len);
                //ESP_LOG_BUFFER_HEXDUMP("STATUS_TX_HEX", pkt, len, ESP_LOG_INFO);
            }

            last_status_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ================= FSM TASK ================= */
static void task_fsm_loop(void *arg)
{
    sm_init();
    sm_start();

    float temp_cache[NUM_PT100] = {0};
    bool float1_cache = false;
    bool float2_cache = false;

    while (1) {
        for (int i = 0; i < NUM_PT100; i++) temp_cache[i] = g_system_data.pt100[i];
        float1_cache = g_system_data.float_1;
        float2_cache = g_system_data.float_2;

        sm_update_temperatures(temp_cache[0], temp_cache[1],1);
        sm_update_floats(float1_cache, float2_cache);
        sm_tick();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



/* ================= TASK CREATION ================= */
static void create_tasks(void)
{
    xTaskCreate(task_sensor_loop, "sensor_task", 4096, NULL, 5, &task_sensors);
    xTaskCreate(task_router_loop, "router_task", 4096, NULL, 4, &task_router);
    xTaskCreate(task_fsm_loop, "fsm_task", 4096, NULL, 6, &task_fsm);
    xTaskCreate(master_uart_rx_task, "uart_rx", 4096, NULL, 6, NULL);
    ESP_LOGI(TAG, "All tasks created");
}

/* ================= PUBLIC INIT ================= */
void app_master_init(void)
{
    ESP_LOGI(TAG, "Master init started");

    // 1. Init I2C + RTC
    ESP_ERROR_CHECK(bus_i2c_init());
    ESP_ERROR_CHECK(rtc_service_init());

    // 2. Init SPI + MAX31865
    ESP_ERROR_CHECK(init_max31865_devices());

    // 3. UART transport
    actuator_uart_init(); 
    
    master_transport_init();
    

    // 4. FSM + Command manager
    cmdmgr_init();
    
    
    float_sensors_init();


    ESP_LOGI(TAG, "Master init complete");
}

/* ================= PUBLIC START ================= */
void app_master_start(void)
{
    ESP_LOGI(TAG, "Starting master tasks");
    create_tasks();
    
		

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


/*
    / ================= STEP 3B: LoRa SPI DEVICE ================= /
    //ESP_ERROR_CHECK(init_lora_spi_device());

    / ================= STEP 3C: LoRa RA-02 ================= /
    ESP_ERROR_CHECK(lora_init(&lora, spi_lora, PIN_LORA_RST));
    lora_set_frequency(&lora, 915000000);   // change if needed
    lora_enable_rx(&lora);
    ESP_LOGI(TAG, "LoRa initialized");
    
    
    
    
    
    
    / Create SPI device for LoRa (VSPI) /
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


*/




/*
void uart_test_task(void *arg)
{
    uint32_t seq = 1;
    uint8_t buf[64];

    while (1) {

        size_t len = protocol_build_command_packet(
            buf,
            sizeof(buf),
            seq,
            CMD_UART_TEST,
            0xA5A5,      // param16 test pattern
            0x12345678   // param32 test pattern
        );

        if (len > 0) {
            //actuator_send_bytes(buf, len);

            ESP_LOGI(TAG,
                     "TX TEST ACTUATOR seq=%lu",
                     seq);
        }

        seq++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

    xTaskCreate(
    uart_test_task,
    "uart_test",
    2048,
    NULL,
    5,
    NULL
);

*/