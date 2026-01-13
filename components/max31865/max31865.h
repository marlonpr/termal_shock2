#pragma once
#include "driver/spi_master.h"
#include "esp_err.h"

typedef enum {
    MAX31865_2WIRE = 2,
    MAX31865_3WIRE = 3,
    MAX31865_4WIRE = 4
} max31865_wire_t;

typedef struct {
    spi_device_handle_t spi;
    max31865_wire_t wire_mode;
    int cs_pin;  // CS pin for this device
} max31865_t;

esp_err_t max31865_init(max31865_t *dev, spi_host_device_t spi_bus, int cs_pin, max31865_wire_t wire_mode);
esp_err_t max31865_read_temperature(max31865_t *dev, float *temperature_c);
