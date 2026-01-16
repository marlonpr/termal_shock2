#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define PROTOCOL_VERSION  1
#define PACKET_MAGIC      0xA5
#define CMD_UART_TEST   0xF0

/* ================= TELEMETRY COMMAND IDS ================= */

/* Periodic (1 Hz) */
#define CMD_TELEMETRY_STATUS       0x30
#define CMD_TELEMETRY_TEMPERATURE  0x31

/* Event-driven */
#define CMD_TELEMETRY_EVENT        0x32



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
    PKT_ACK     = 0x11,
    
    PKT_TELEM_STATE  = 0x20,
    PKT_TELEM_TEMP   = 0x21,
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

    /* Temperatures (°C × 100) */
    int16_t  pt100_1_centi;
    int16_t  pt100_2_centi;
    int16_t  pt100_3_centi;

    /* Digital inputs */
    uint8_t  float_mask;     // bit0=float1, bit1=float2

    /* State machine */
    uint8_t  sm_state;       // sm_state_t
    uint8_t  ts_state;       // ts_state_t
    uint8_t  mode;           // MODE_HOT / MODE_COLD
    uint8_t  reserved1;

    /* Timing / counters */
    uint32_t elapsed_sec;    // seconds since current state entry
    uint32_t cycle_count;    // completed cycles

} payload_status_t;
#pragma pack(pop)


/* ================= EVENT IDS ================= */

typedef enum {
    EVT_TEST_STARTED   = 0x01,
    EVT_TEST_FINISHED  = 0x02,
    EVT_CYCLE_COMPLETE = 0x03,
    EVT_FLOAT_CHANGED  = 0x04,
    EVT_STATE_CHANGED  = 0x05
} event_id_t;



/* ================= EVENT PAYLOAD ================= */
#pragma pack(push, 1)
typedef struct {
    uint8_t  event_id;   // event_id_t
    uint8_t  reserved;
    uint16_t reserved2;
    uint32_t value;      // event-specific meaning
} payload_event_t;
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

size_t protocol_build_telemetry_packet(uint8_t *buf,
                                       size_t buf_size,
                                       uint8_t pkt_type,
                                       uint32_t sequence,
                                       const void *payload,
                                       uint16_t payload_len);
                                       
                                       
                                       #define PROTOCOL_MAX_PAYLOAD      64
#define PROTOCOL_HEADER_SIZE      8
#define PROTOCOL_CRC_SIZE         2
#define PROTOCOL_MAX_PACKET       (PROTOCOL_HEADER_SIZE + PROTOCOL_MAX_PAYLOAD + PROTOCOL_CRC_SIZE)

/* ============================================================
 *  COMMAND / TELEMETRY IDS
 * ============================================================ */

/* ---- Telemetry: periodic or on-change ---- */
typedef enum {
    CMD_TELEM_TEMP          = 0x10,   /* PT100 temperature */
    CMD_TELEM_STATUS        = 0x21,   /* state / mode / elapsed / cycles */
    CMD_TELEM_FLOAT_STATE   = 0x22,   /* float switch (edge only) */
} protocol_telem_cmd_t;

/* ---- Control / events ---- */
typedef enum {
    CMD_EVENT_TEST_STARTED  = 0x30,
    CMD_EVENT_TEST_FINISHED = 0x31,
    CMD_EVENT_CYCLE_DONE    = 0x32,
} protocol_event_cmd_t;

/* ============================================================
 *  PACKET HEADER (WIRE FORMAT)
 * ============================================================ */

/*
 *  Byte layout:
 *  ---------------------------------------------------------
 *  0   : magic      (0xA5)
 *  1   : version    (0x01)
 *  2   : cmd
 *  3   : payload_len
 *  4-7 : sequence (LE)
 *  8.. : payload
 *  N   : CRC16 (LE)
 */
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  cmd;
    uint8_t  payload_len;
    uint32_t seq;
} protocol_header_t;

/* ============================================================
 *  TELEMETRY PAYLOADS
 * ============================================================ */

/* ---- Temperature telemetry ---- */
typedef struct __attribute__((packed)) {
    uint8_t sensor_id;        /* 0..N */
    float   temperature_c;   /* IEEE754 */
} telem_temp_t;
/* payload_len = 5 */

/* ---- System status (periodic) ---- */
typedef struct __attribute__((packed)) {
    uint8_t  mode;           /* 0=ModeA, 1=ModeB */
    uint8_t  state;          /* enum system_state */
    uint16_t reserved;
    uint32_t elapsed_sec;    /* RTC elapsed seconds */
    uint32_t cycle_count;    /* completed cycles */
} telem_status_t;
/* payload_len = 12 */

/* ---- Float switch (edge-triggered) ---- */
typedef struct __attribute__((packed)) {
    uint8_t float_id;        /* which float */
    uint8_t state;           /* 0=open, 1=closed */
} telem_float_state_t;
/* payload_len = 2 */

/* ============================================================
 *  EVENT PAYLOADS
 * ============================================================ */

typedef struct __attribute__((packed)) {
    uint32_t value;          /* generic counter / timestamp */
} event_u32_t;
/* payload_len = 4 */

/* ============================================================
 *  CRC
 * ============================================================ */

uint16_t protocol_crc16(const uint8_t *data, size_t len);

/* ============================================================
 *  PACKET BUILDERS
 * ============================================================ */

/*
 * Builds a telemetry or event packet.
 * Returns total packet length (header + payload + crc)
 */
size_t protocol_build_packet(
    uint8_t       *out_buf,
    size_t         out_buf_size,
    uint8_t        cmd,
    uint32_t       seq,
    const void    *payload,
    uint8_t        payload_len
);

/* ============================================================
 *  PACKET PARSER
 * ============================================================ */

typedef struct {
    protocol_header_t header;
    const uint8_t    *payload;
    uint16_t          crc_rx;
    uint16_t          crc_calc;
} protocol_packet_t;

/*
 * Validates framing + CRC.
 * Returns 1 if valid, 0 if invalid.
 */
int protocol_parse_packet(
    const uint8_t       *buf,
    size_t               len,
    protocol_packet_t   *out
); 