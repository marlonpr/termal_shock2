#include "rtc_service.h"
#include "bus.h"
#include "esp_log.h"

// Define DS3231 I2C address if not already defined
#ifndef DS3231_ADDR
#define DS3231_ADDR 0x68
#endif

static const char *TAG = "RTC_SERVICE";

// DS3231 device struct
static ds3231_dev_t rtc_dev;

// Initialize RTC service
esp_err_t rtc_service_init(void)
{
    ESP_LOGI(TAG, "Initializing RTC service...");

    if (!i2c_bus) {
        ESP_ERROR_CHECK(bus_i2c_init());
    }

    esp_err_t ret = ds3231_init(&rtc_dev, i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DS3231 init failed: %d", ret);
        return ret;
    }

    // Probe device to ensure it's online
    ret = i2c_master_probe(rtc_dev.bus, DS3231_ADDR, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DS3231 not responding at I2C address 0x%02X", DS3231_ADDR);
        rtc_dev.dev = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "DS3231 detected at I2C bus %p", rtc_dev.bus);
    
    
    
	    // ===== Check RTC time and set if needed =====
	ds3231_time_t now;
	if (ds3231_get_time(&rtc_dev, &now) == ESP_OK) {
	    ESP_LOGI(TAG, "RTC current time: %04d-%02d-%02d %02d:%02d:%02d",
	             now.year, now.month, now.day, now.hour, now.minute, now.second);
	
	    if (now.year < 2025) {
	        ds3231_time_t set_time = {
	            .year        = 2026,
	            .month       = 1,
	            .day         = 12,
	            .hour        = 14,
	            .minute      = 52,
	            .second      = 0,
	            .day_of_week = 2 // Tuesday
	        };
	        ESP_LOGI(TAG, "RTC year < 2025, setting new time...");
	        ESP_ERROR_CHECK(ds3231_set_time(&rtc_dev, &set_time));
	    }
	} else {
	    ESP_LOGW(TAG, "Failed to read RTC time during init");
	}
    
    
    
    
    return ESP_OK;
}

// Safe read
esp_err_t rtc_service_get_time(ds3231_time_t *time)
{
    if (!rtc_dev.dev) {
        ESP_LOGW(TAG, "RTC not initialized, returning default time");
        time->year = 2000;
        time->month = 1;
        time->day = 1;
        time->hour = 0;
        time->minute = 0;
        time->second = 0;
        return ESP_FAIL;
    }

    return ds3231_get_time(&rtc_dev, time);
}

// Convert DS3231 time to epoch seconds (since 2000-01-01)
uint32_t rtc_get_epoch_seconds(void)
{
    ds3231_time_t t;
    if (rtc_service_get_time(&t) != ESP_OK) {
        return 0; // fallback
    }

    int days_in_month[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    uint32_t days = 0;

    for (int y = 2000; y < t.year; y++) {
        days += 365 + ((y % 4 == 0) ? 1 : 0);
    }
    for (int m = 1; m < t.month; m++) {
        days += days_in_month[m - 1];
        if (m == 2 && t.year % 4 == 0) days++;
    }
    days += t.day - 1;

    return days * 86400 + t.hour * 3600 + t.minute * 60 + t.second;
}


