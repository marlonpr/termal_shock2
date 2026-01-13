#pragma once

#include "esp_err.h"
#include "ds3231.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize RTC service
esp_err_t rtc_service_init(void);

// Read current time safely
esp_err_t rtc_service_get_time(ds3231_time_t *time);

// Return epoch seconds since 2000-01-01 00:00:00
uint32_t rtc_get_epoch_seconds(void);

#ifdef __cplusplus
}
#endif
