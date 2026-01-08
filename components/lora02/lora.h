#pragma once
#include "esp_err.h"
#include "driver/spi_master.h"
#include <stdint.h>
#include "driver/gpio.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    spi_device_handle_t spi;
    gpio_num_t rst_pin;
} lora_dev_t;

/* Driver API */
esp_err_t lora_init(
    lora_dev_t *dev,
    spi_device_handle_t spi,
    gpio_num_t rst_pin
);

void lora_set_frequency(lora_dev_t *dev, long freq);
void lora_send_packet(lora_dev_t *dev, const uint8_t *data, int len);
void lora_enable_rx(lora_dev_t *dev);
int  lora_receive_packet(lora_dev_t *dev, uint8_t *buf, int maxlen);

#ifdef __cplusplus
}
#endif
