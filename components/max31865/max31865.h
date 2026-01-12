#pragma once

#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MAX31865_2WIRE = 0,
    MAX31865_3WIRE,
    MAX31865_4WIRE
} max31865_wire_t;

typedef struct {
    spi_device_handle_t spi;
    max31865_wire_t wire_mode;
} max31865_t;

/* API */
esp_err_t max31865_init(
    max31865_t *dev,
    spi_device_handle_t spi,
    max31865_wire_t wire_mode
);

esp_err_t max31865_read_temperature(
    max31865_t *dev,
    float *temperature_c
);

#ifdef __cplusplus
}
#endif
