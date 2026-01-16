#pragma once
#include <stddef.h>
#include <stdint.h>

void master_transport_init(void);
void master_send_bytes(const uint8_t *data, size_t len);
void actuator_uart_init(void);

void actuator_send_bytes(const uint8_t *data, size_t len);
void actuator_send_command(uint8_t cmd_id, uint16_t param16, uint32_t param32);
void master_send_bytes_2(const uint8_t *data, size_t len);