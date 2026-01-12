// protocol.c
#include "protocol.h"
#include "app_data.h"
#include "crc16.h"
#include <string.h>

/* ================= INTERNAL ================= */

static uint32_t local_sequence = 0;

/* ================= HELPERS ================= */

static inline size_t frame_len(size_t payload_len)
{
    return sizeof(packet_header_t) + payload_len + 2; // + CRC16
}

static void append_crc(uint8_t *buf, size_t len_no_crc)
{
    uint16_t crc = crc16_ccitt(buf, len_no_crc, 0);
    buf[len_no_crc]     = (crc >> 8) & 0xFF;
    buf[len_no_crc + 1] = crc & 0xFF;
}

/* ================= STATUS ================= */

size_t protocol_build_status_packet(uint8_t *out, size_t max_len)
{
    if (!out) return 0;

    const size_t payload_len = sizeof(payload_status_t);
    const size_t total_len   = frame_len(payload_len);

    if (max_len < total_len) return 0;

    packet_header_t *hdr = (packet_header_t *)out;
    payload_status_t *pl =
        (payload_status_t *)(out + sizeof(packet_header_t));

    /* Payload */
    pl->pt100_1_centi = (int16_t)(g_system_data.pt100_1 * 100.0f);
    pl->pt100_2_centi = (int16_t)(g_system_data.pt100_2 * 100.0f);
    pl->pt100_3_centi = (int16_t)(g_system_data.pt100_3 * 100.0f);

    pl->float_mask =
        (g_system_data.float_1 ? 0x01 : 0) |
        (g_system_data.float_2 ? 0x02 : 0);

    /* Header */
    hdr->magic     = PACKET_MAGIC;
    hdr->version   = PROTOCOL_VERSION;
    hdr->type      = PKT_STATUS;
    hdr->length    = payload_len;
    hdr->sequence  = local_sequence++;
    hdr->timestamp =
        g_system_data.second +
        g_system_data.minute * 60 +
        g_system_data.hour   * 3600;

    /* CRC */
    append_crc(out, sizeof(packet_header_t) + payload_len);

    return total_len;
}

/* ================= COMMAND ================= */

size_t protocol_build_command_packet(uint8_t *out, size_t max_len,
                                     uint32_t sequence,
                                     uint8_t  cmd_id,
                                     uint16_t param16,
                                     uint32_t param32)
{
    if (!out) return 0;

    const size_t payload_len = sizeof(payload_command_t);
    const size_t total_len   = frame_len(payload_len);

    if (max_len < total_len) return 0;

    packet_header_t *hdr = (packet_header_t *)out;
    payload_command_t *pl =
        (payload_command_t *)(out + sizeof(packet_header_t));

    /* Payload */
    pl->cmd_id  = cmd_id;
    pl->flags   = 0;
    pl->param16 = param16;
    pl->param32 = param32;

    /* Header */
    hdr->magic     = PACKET_MAGIC;
    hdr->version   = PROTOCOL_VERSION;
    hdr->type      = PKT_COMMAND;
    hdr->length    = payload_len;
    hdr->sequence  = sequence;
    hdr->timestamp = 0;

    /* CRC */
    append_crc(out, sizeof(packet_header_t) + payload_len);

    return total_len;
}

/* ================= ACK ================= */

size_t protocol_build_ack_packet(uint8_t *out, size_t max_len,
                                 uint32_t sequence,
                                 uint8_t  cmd_id,
                                 uint8_t  status)
{
    if (!out) return 0;

    const size_t payload_len = sizeof(payload_ack_t);
    const size_t total_len   = frame_len(payload_len);

    if (max_len < total_len) return 0;

    packet_header_t *hdr = (packet_header_t *)out;
    payload_ack_t *ack =
        (payload_ack_t *)(out + sizeof(packet_header_t));

    /* Payload */
    ack->cmd_id   = cmd_id;
    ack->status   = status;
    ack->reserved = 0;

    /* Header */
    hdr->magic     = PACKET_MAGIC;
    hdr->version   = PROTOCOL_VERSION;
    hdr->type      = PKT_ACK;
    hdr->length    = payload_len;
    hdr->sequence  = sequence;
    hdr->timestamp = 0;

    /* CRC */
    append_crc(out, sizeof(packet_header_t) + payload_len);

    return total_len;
}
