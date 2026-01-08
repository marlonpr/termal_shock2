#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdint.h>

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
} ds3231_dev_t;

typedef struct {
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day_of_week;  // 1–7
    uint8_t day;
    uint8_t month;
    uint16_t year;        // full year (e.g. 2026)
} ds3231_time_t;

/* Driver API */
esp_err_t ds3231_init(
    ds3231_dev_t *dev,
    i2c_master_bus_handle_t bus
);

esp_err_t ds3231_set_time(ds3231_dev_t *dev, const ds3231_time_t *time);
esp_err_t ds3231_get_time(ds3231_dev_t *dev, ds3231_time_t *time);
