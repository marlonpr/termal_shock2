#include "master_telemetry.h"
#include "protocol.h"
#include "telemetry.h"
#include "master_link.h"

static uint32_t telem_seq = 1;

/* ---------------- STATE TELEMETRY ---------------- */

void master_send_telem_state(const telemetry_status_t *st)
{
    uint8_t buf[64];

    size_t len = protocol_build_telemetry_packet(
        buf,
        sizeof(buf),
        PKT_TELEM_STATE,
        telem_seq++,
        st,
        sizeof(*st)
    );

    if (len > 0) {
        master_link_send_bytes(buf, len);
    }
}

/* ---------------- TEMPERATURE TELEMETRY ---------------- */

void master_send_telem_temp(const telemetry_temperature_t *tp)
{
    uint8_t buf[64];

    size_t len = protocol_build_telemetry_packet(
        buf,
        sizeof(buf),
        PKT_TELEM_TEMP,
        telem_seq++,
        tp,
        sizeof(*tp)
    );

    if (len > 0) {
        master_link_send_bytes(buf, len);
    }
}

