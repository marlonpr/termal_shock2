#pragma once
#include "telemetry.h"

void master_send_telem_state(const telemetry_status_t *st);
void master_send_telem_temp(const telemetry_temperature_t *tp);
