#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "board_pins.h"

/* Global I2C bus */
extern i2c_master_bus_handle_t i2c_bus;

/* Global SPI devices */
extern spi_device_handle_t spi_lora;

/* API */
esp_err_t bus_i2c_init(void);
esp_err_t bus_spi_init(void);
