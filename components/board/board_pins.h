#pragma once

/* ================= I2C BUS ================= */
#define I2C_MASTER_PORT   I2C_NUM_0
#define I2C_SDA_PIN       PIN_I2C_SDA
#define I2C_SCL_PIN       PIN_I2C_SCL

/* ================= SPI BUS ================= */
#define SPI_HOST_USED     SPI3_HOST   // VSPI on ESP32
#define SPI_MOSI_PIN      PIN_MAX31865_MOSI
#define SPI_MISO_PIN      PIN_MAX31865_MISO
#define SPI_SCK_PIN       PIN_MAX31865_SCK


/* ================= SPI — MAX31865 (HSPI) ================= */
#define PIN_MAX31865_SCK   14 //SCK
#define PIN_MAX31865_MOSI  13 //SDI
#define PIN_MAX31865_MISO  12 //SDO
#define PIN_MAX31865_CS1   16
#define PIN_MAX31865_CS2   17
#define PIN_MAX31865_CS3   32



/* ================= I2C — DS3231 ================= */
#define PIN_I2C_SCL   22
#define PIN_I2C_SDA   21

/* ================= UART ROUTING ================= */
#define UART_UI       UART_NUM_1
#define UART_RELAY    UART_NUM_2

#define PIN_UART_UI_TX     4 //4
#define PIN_UART_UI_RX     15 //15

#define PIN_UART_RELAY_TX   19
#define PIN_UART_RELAY_RX  18

/* ================= FLOAT SENSORS ================= */
#define PIN_FLOAT_1   35
#define PIN_FLOAT_2   34
