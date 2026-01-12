#include "rtc_service.h"
#include "ds3231.h"
#include "esp_log.h"
#include <time.h>  // for mktime

/* Declare global RTC device defined elsewhere (rtc_device.c) */
extern ds3231_dev_t rtc;

static const char *TAG = "RTC_SERVICE";

/* ================= RTC EPOCH TIME ================= */

uint32_t rtc_get_epoch_seconds(void)
{
    ds3231_time_t rtc_time;

    if (ds3231_get_time(&rtc, &rtc_time) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read RTC");
        return 0;
    }

    struct tm tm_time = {0};

    /* Map DS3231 fields to struct tm */
    tm_time.tm_sec  = rtc_time.second;
    tm_time.tm_min  = rtc_time.minute;
    tm_time.tm_hour = rtc_time.hour;
    tm_time.tm_mday = rtc_time.day;
    tm_time.tm_mon  = rtc_time.month - 1;  // tm_mon: 0-11
    tm_time.tm_year = rtc_time.year + 100; // tm_year: years since 1900

    return (uint32_t)mktime(&tm_time);
}

/* ================= RTC HELPER: INDIVIDUAL FIELDS ================= */

bool rtc_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second,
                  uint8_t *day, uint8_t *month, uint16_t *year)
{
    ds3231_time_t rtc_time;

    if (ds3231_get_time(&rtc, &rtc_time) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read RTC");
        return false;
    }

    if (hour)   *hour   = rtc_time.hour;
    if (minute) *minute = rtc_time.minute;
    if (second) *second = rtc_time.second;
    if (day)    *day    = rtc_time.day;
    if (month)  *month  = rtc_time.month;
    if (year)   *year   = rtc_time.year + 2000;

    return true;
}
