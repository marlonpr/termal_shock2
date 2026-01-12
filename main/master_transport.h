#pragma once
#include <stddef.h>
#include <stdint.h>

void master_transport_init(void);
void master_send_bytes(const uint8_t *data, size_t len);
