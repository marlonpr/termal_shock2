#pragma once
#pragma once
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"

/* ================= I2C BUS ================= */
#define I2C_MASTER_PORT   I2C_NUM_0
#define I2C_SDA_PIN       PIN_I2C_SDA
#define I2C_SCL_PIN       PIN_I2C_SCL

/* ================= SPI BUS ================= */
#define SPI_HOST_USED     SPI3_HOST   // VSPI on ESP32
#define SPI_MOSI_PIN      PIN_LORA_MOSI
#define SPI_MISO_PIN      PIN_LORA_MISO
#define SPI_SCK_PIN       PIN_LORA_SCK


/* ================= SPI — MAX31865 (HSPI) ================= */
#define PIN_MAX_SCK   14
#define PIN_MAX_MOSI  13
#define PIN_MAX_MISO  12
#define PIN_MAX_CS1   16
#define PIN_MAX_CS2   17
#define PIN_MAX_CS3   21

/* ================= SPI — LORA (VSPI) ================= */
#define PIN_LORA_SCK   18
#define PIN_LORA_MOSI  23
#define PIN_LORA_MISO  19
#define PIN_LORA_CS     5
#define PIN_LORA_RST   27
#define PIN_LORA_DIO0  26

/* ================= I2C — DS3231 ================= */
#define PIN_I2C_SCL   22
#define PIN_I2C_SDA   25

/* ================= UART ROUTING ================= */
#define UART_UI       UART_NUM_2
#define UART_RELAY    UART_NUM_1

#define PIN_UART_UI_TX     33
#define PIN_UART_UI_RX     32

#define PIN_UART_RELAY_TX   4
#define PIN_UART_RELAY_RX  15

/* ================= FLOAT SENSORS ================= */
#define PIN_FLOAT_1   34
#define PIN_FLOAT_2   35
