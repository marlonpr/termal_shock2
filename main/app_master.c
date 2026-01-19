// app_master.c
#include "app_master.h"
#include "app_data.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bus.h"
#include "ds3231.h"
#include "max31865.h"
#include "board_pins.h"

#include "state_machine.h"
#include "termal_shock.h"
#include "rtc_service.h"

#include "command_manager.h"
#include "master_transport.h"
#include "master_link.h"
#include "master_uart_rx.h"
#include "protocol.h"

sm_ctx_t ctx;



static const char *TAG7 = "APP_MASTER";

void send_cycles(int cycles)
{
    char tx_buf[32];

    int n = snprintf(tx_buf, sizeof(tx_buf),
                     "CYCLES=%d\n", cycles);

    if (n > 0) {
        uart_write_bytes(UART_UI, tx_buf, n);
        ESP_LOGI("TX", "Sent: %s", tx_buf);
    }
}




// number of cycles i mastr_command 	    ts_data.max_cycles  = 3;




//int cycles = ts_data.cycle_count;

void uart_send_string(const char *str)
{
    size_t len = strlen(str);
    uart_write_bytes(UART_UI, str, len);
}



/* ============================================================
 * PT100 / MAX31865
 * ============================================================ */

#define NUM_PT100 2

static max31865_t max31865_dev[NUM_PT100];
static const int max31865_cs_pins[NUM_PT100] = {
    PIN_MAX31865_CS1,
    PIN_MAX31865_CS2
};

static esp_err_t init_max31865_devices(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MAX31865_MISO,
        .mosi_io_num = PIN_MAX31865_MOSI,
        .sclk_io_num = PIN_MAX31865_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    ESP_ERROR_CHECK(
        spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO)
    );

    for (int i = 0; i < NUM_PT100; i++) {
        ESP_ERROR_CHECK(
            max31865_init(
                &max31865_dev[i],
                SPI2_HOST,
                max31865_cs_pins[i],
                MAX31865_3WIRE
            )
        );
    }

    return ESP_OK;
}

/* ============================================================
 * SENSOR TASK (1 Hz)
 * ============================================================ */

static void task_sensor_loop(void *arg)
{
    ds3231_time_t last_time = {0};
    float last_temp[NUM_PT100] = {0};

    while (1) {

        rtc_service_get_time(&last_time);
        g_system_data.second = last_time.second;

        /* -------- Read PT100 -------- */
        for (int i = 0; i < NUM_PT100; i++) {
            float t;
            if (max31865_read_temperature(&max31865_dev[i], &t) == ESP_OK) {
                last_temp[i] = t;
            }
            g_system_data.pt100[i] = last_temp[i];
        }

        /* -------- Log once per second -------- */
        ESP_LOGI("TEMP",
                 "T1=%.2f T2=%.2f T3=%.2f T4=%.2f",
                 g_system_data.pt100[0],
                 g_system_data.pt100[1],
                 g_system_data.pt100[2],
                 g_system_data.pt100[3]);
           
		//master_send_bytes_2((const uint8_t *)"1234", 5);


		//============================= SEND TEMP ===============================//            
		char tx_buf[32];
		int n = snprintf(tx_buf, sizeof(tx_buf),
		                 "T1=%.2f\n",
		                 g_system_data.pt100[0]);
		
		uart_write_bytes(UART_UI, tx_buf, n);
		//================================================================//
  
        

       
             
        

		

		
						//============================= SEND CYCLE ===============================//
		        char tx_buf2[32];
				int n2 = snprintf(tx_buf2, sizeof(tx_buf2),
				                 "CYCLES=%lu\n",
				                 (unsigned long)ts_data.cycle_count);
				                 
				                         ESP_LOGI("TX", "Sending: %s", tx_buf2);
				        ESP_LOGI("TX", "Sending: %s", tx_buf2);
				
				
				if (n2 > 0) {
				    uart_write_bytes(UART_UI, tx_buf2, n2);
				}
				//================================================================//
       

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


/* ============================================================
 * CONTROL LOOP (RTC-BASED FSM TICK)
 * ============================================================ */

static void task_control_loop(void *arg)
{
    uint32_t last_sec = rtc_get_epoch_seconds();

    while (1) {

        uint32_t now = rtc_get_epoch_seconds();
        if (now != last_sec) {
            last_sec = now;

            if (ts_data.state == TS_RUNNING) {
                sm_tick();   // << deterministic, RTC-driven
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ============================================================
 * ROUTER TASK
 * ============================================================ */

static void task_router_loop(void *arg)
{
    uint32_t last_status_ms = 0;

    while (1) {
        uint32_t now_ms = esp_timer_get_time() / 1000;
        master_link_tick(now_ms);

        if (now_ms - last_status_ms >= 500) {
            uint8_t pkt[64];
            size_t len = protocol_build_status_packet(pkt, sizeof(pkt));
            if (len > 0) master_send_bytes(pkt, len);
            last_status_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ============================================================
 * TASK CREATION
 * ============================================================ */

static void create_tasks(void)
{
    xTaskCreate(task_sensor_loop,  "sensor_task",  4096, NULL, 5, NULL);
    xTaskCreate(task_control_loop, "control_task", 4096, NULL, 6, NULL);
    xTaskCreate(task_router_loop,  "router_task",  4096, NULL, 4, NULL);
    xTaskCreate(master_uart_rx_task,"uart_rx",     4096, NULL, 6, NULL);

    ESP_LOGI(TAG7, "All tasks created");
}

/* ============================================================
 * PUBLIC INIT / START
 * ============================================================ */

void app_master_init(void)
{
    ESP_LOGI(TAG7, "Master init started");

    ESP_ERROR_CHECK(bus_i2c_init());
    ESP_ERROR_CHECK(rtc_service_init());
    ESP_ERROR_CHECK(init_max31865_devices());

    actuator_uart_init();
    master_transport_init();
    cmdmgr_init();

    sm_init();
    
    thermal_shock_init(3);

    ESP_LOGI(TAG7, "Master init complete");
}

void app_master_start(void)
{
    ESP_LOGI(TAG7, "Starting master tasks");
    create_tasks();
}











    //xTaskCreate(task_lora_loop,   "lora_task",   4096, NULL, 3, &task_lora);

/*
static void task_lora_loop(void *arg)
{
    while (1) {
        ESP_LOGI(TAG7, "LoRa handler");
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
    ESP_LOGI(TAG7, "LoRa initialized");
    
    
    
    
    
    
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

            ESP_LOGI(TAG7,
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



/*

		if(ts_data.state == TS_RUNNING)
		{
			//============================= SEND SEC ===============================//
			char tx_buf3[32];
			int e_sec = sm_elapsed_sec();		
			int n3 = snprintf(tx_buf3, sizeof(tx_buf3),
			                 "E_SEC=%lu\n",
			                 (unsigned long)e_sec);
			                 
			                         ESP_LOGI("TX", "Sending: %s", tx_buf3);
			        ESP_LOGI("TX", "Sending: %s", tx_buf3);
			
			
			if (n3 > 0) {
			    uart_write_bytes(UART_UI, tx_buf3, n3);
			}
			//================================================================//
			
			

		}
*/