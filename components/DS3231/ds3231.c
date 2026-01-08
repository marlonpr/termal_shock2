#include "ds3231.h"
#include "esp_log.h"

#define DS3231_ADDR 0x68
#define TAG "DS3231"

/* BCD helpers */
static inline uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

static inline uint8_t bcd2dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

esp_err_t ds3231_init(
    ds3231_dev_t *dev,
    i2c_master_bus_handle_t bus
)
{
    if (!dev || !bus) return ESP_ERR_INVALID_ARG;

    dev->bus = bus;

    i2c_device_config_t dev_cfg = {
        .device_address = DS3231_ADDR,
        .scl_speed_hz = 100000
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(bus, &dev_cfg, &dev->dev)
    );

    ESP_LOGI(TAG, "DS3231 device attached");
    return ESP_OK;
}

esp_err_t ds3231_set_time(ds3231_dev_t *dev, const ds3231_time_t *time)
{
    if (!dev || !time) return ESP_ERR_INVALID_ARG;

    uint8_t buf[8] = {
        0x00,
        dec2bcd(time->second),
        dec2bcd(time->minute),
        dec2bcd(time->hour & 0x3F),
        dec2bcd(time->day_of_week & 0x07),
        dec2bcd(time->day),
        dec2bcd(time->month),
        dec2bcd(time->year % 100)
    };

    return i2c_master_transmit(dev->dev, buf, sizeof(buf), -1);
}

esp_err_t ds3231_get_time(ds3231_dev_t *dev, ds3231_time_t *time)
{
    if (!dev || !time) return ESP_ERR_INVALID_ARG;

    uint8_t reg = 0x00;
    uint8_t data[7];

    ESP_ERROR_CHECK(i2c_master_transmit(dev->dev, &reg, 1, -1));
    ESP_ERROR_CHECK(i2c_master_receive(dev->dev, data, 7, -1));

    time->second      = bcd2dec(data[0] & 0x7F);
    time->minute      = bcd2dec(data[1] & 0x7F);
    time->hour        = bcd2dec(data[2] & 0x3F);
    time->day_of_week = bcd2dec(data[3] & 0x07);
    time->day         = bcd2dec(data[4] & 0x3F);
    time->month       = bcd2dec(data[5] & 0x1F);
    time->year        = 2000 + bcd2dec(data[6]);

    return ESP_OK;
}
