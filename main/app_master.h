#include <stdbool.h>

#pragma once

void app_master_init(void);
void app_master_start(void);
void master_send_relay_command(bool hot_on, bool cold_on);

