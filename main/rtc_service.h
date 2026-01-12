#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Returns current epoch seconds from DS3231 RTC */
uint32_t rtc_get_epoch_seconds(void);

/* Optional: read individual time/date components */
bool rtc_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second,
                  uint8_t *day, uint8_t *month, uint16_t *year);
