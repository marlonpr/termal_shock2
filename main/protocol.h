#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define PROTOCOL_VERSION  1
#define PACKET_MAGIC      0xA5

/* ================= MODE DEFINITIONS (WIRE LEVEL) ================= */
/* Used by CMD_SET_MODE, CMD_FORCE_RELAY */

#define MODE_OFF   0x00
#define MODE_HOT   0x01
#define MODE_COLD  0x02


/* ================= PACKET TYPES ================= */
typedef enum {
    PKT_STATUS  = 0x01,    // Master → others
    PKT_EVENT   = 0x02,

    PKT_COMMAND = 0x10,    // Others → master
    PKT_ACK     = 0x11
} packet_type_t;

/* ================= WIRE PACKET HEADER ================= */
#pragma pack(push, 1)
typedef struct {
    uint8_t  magic;        // 0xA5
    uint8_t  version;      // PROTOCOL_VERSION
    uint8_t  type;         // packet_type_t
    uint8_t  length;       // payload length (bytes)

    uint32_t sequence;     // monotonically increasing
    uint32_t timestamp;    // unix time (seconds)

    /* Payload follows */
} packet_header_t;
#pragma pack(pop)

/* ================= STATUS PAYLOAD ================= */
#pragma pack(push, 1)
typedef struct {
    int16_t  pt100_1_centi;   // temperature * 100
    int16_t  pt100_2_centi;
    int16_t  pt100_3_centi;

    uint8_t  float_mask;     // bit0=float1, bit1=float2
} payload_status_t;
#pragma pack(pop)

/* ================= COMMAND DEFINITIONS ================= */

/* Command IDs */
typedef enum {
    CMD_NOP           = 0x00,
    CMD_START_TEST    = 0x01,
    CMD_STOP_TEST     = 0x02,
    CMD_SET_MODE      = 0x03,
    CMD_FORCE_RELAY   = 0x04,
    CMD_SYNC_TIME     = 0x05,
    CMD_REQUEST_STATE = 0x06
} command_id_t;

/* Command payload */
#pragma pack(push, 1)
typedef struct {
    uint8_t  cmd_id;      // command_id_t
    uint8_t  flags;       // reserved
    uint16_t param16;     // command-specific
    uint32_t param32;     // command-specific
} payload_command_t;
#pragma pack(pop)

/* ================= ACK PAYLOAD ================= */
#pragma pack(push, 1)
typedef struct {
    uint8_t  cmd_id;      // echoed command
    uint8_t  status;      // 0=OK, nonzero=error
    uint16_t reserved;
} payload_ack_t;
#pragma pack(pop)

/* ================= ACK STATUS CODES ================= */
#define ACK_OK              0x00
#define ACK_ERR_UNKNOWN     0x01
#define ACK_ERR_INVALID     0x02
#define ACK_ERR_EXEC_FAIL   0x03

/* ================= API ================= */

/* Master → Others */
size_t protocol_build_status_packet(
    uint8_t *out,
    size_t max_len
);

/* Others → Master */
size_t protocol_build_command_packet(
    uint8_t *out,
    size_t max_len,
    uint32_t sequence,
    uint8_t  cmd_id,
    uint16_t param16,
    uint32_t param32
);

/* ACK packet (Master → Others) */
size_t protocol_build_ack_packet(
    uint8_t *out,
    size_t max_len,
    uint32_t sequence,
    uint8_t  cmd_id,
    uint8_t  status
);

