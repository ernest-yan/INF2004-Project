/**
 * @file    tlm_codec.h
 * @brief   Telemetry wire format: bit packing, frame encoders, command validator.
 *
 * Pure C (no Pico SDK, no RTOS, no lwIP), so it can be unit-tested on a laptop.
 * Lengths are uint32_t, not size_t: the T-Kernel headers define their own
 * size_t, so this header must not pull in <stddef.h>.
 * All multi-bit fields are written most-significant-bit first.
 * Field widths come from the "Minimum-Bit Data Representation" table in
 * MQTT_PROTOCOL.md. Change C, dashboard.py and that file together.
 */
#ifndef TLM_CODEC_H
#define TLM_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#define TLM_PROTOCOL_VERSION    (1u)

/* Encoded frame sizes in bytes (bits rounded up to whole bytes). */
#define TLM_ONLINE_LEN          (3u)    /*  24 bits */
#define TLM_STATUS_LEN          (21u)   /* 166 bits */
#define TLM_HEARTBEAT_LEN       (11u)   /*  86 bits */
#define TLM_ACK_LEN             (7u)    /*  52 bits */
#define TLM_EVENT_MAX_LEN       (10u)   /*  78 bits, obstacle profile */
#define TLM_CMD_LEN             (5u)    /*  40 bits */
#define TLM_CMD_MAP_LEN         (8u)    /*  64 bits: header + 4 x 6-bit letters */
#define TLM_MAP_LETTERS         (4u)    /* forward, U-turn, left, right */
#define TLM_LETTER_BITS         (6u)    /* letter code: 0 none, 1-26 A-Z, 27-36 0-9 */

/* ---------- Commands (dashboard -> car) ---------- */
typedef enum
{
    TLM_CMD_NONE  = 0,
    TLM_CMD_START = 1,
    TLM_CMD_STOP  = 2,
    TLM_CMD_ESTOP = 3,
    TLM_CMD_RESET = 4,  /* clear brake / S4 lift / S5 fault, only when still */
    TLM_CMD_MAP   = 5   /* barcode letter -> movement mapping (map[]) */
} tlm_cmd_code_t;

typedef enum
{
    TLM_RES_ACCEPTED    = 0,
    TLM_RES_BAD_LEN     = 1,
    TLM_RES_BAD_VERSION = 2,
    TLM_RES_BAD_SESSION = 3,   /* stale (old connection) or retained command */
    TLM_RES_BAD_CMD     = 4,
    TLM_RES_STALE_SEQ   = 5,   /* replay or QoS 1 duplicate */
    TLM_RES_QUEUE_FULL  = 6    /* valid, but qVehicleEvent had no space */
} tlm_result_t;

typedef struct
{
    uint8_t  code;      /* tlm_cmd_code_t */
    uint16_t session;
    uint16_t seq;
    char     map[TLM_MAP_LETTERS];  /* TLM_CMD_MAP only: 'A'-'Z' / '0'-'9',
                                       all different */
} tlm_cmd_t;

/* ---------- Status frame (car -> dashboard, 5 Hz) ---------- */
typedef struct
{
    uint16_t session;
    uint8_t  seq;
    uint8_t  vehicle_state;     /* 3 bits */
    bool     b_estop_latched;   /* 1 bit  */
    uint8_t  line_state;        /* 2 bits */
    uint8_t  last_barcode;      /* 3 bits */
    uint16_t speed_l_mm_s;      /* 10 bits, 0..1023, saturates */
    uint16_t speed_r_mm_s;      /* 10 bits */
    uint16_t enc_l;             /* 16 bits, wraps */
    uint16_t enc_r;             /* 16 bits, wraps */
    uint16_t distance_cm;       /* 12 bits, 0..4095, saturates */
    int8_t   pitch_deg;         /* 7 bits, -64..63, sent as value + 64 */
    uint8_t  hump_max_mm;       /* 8 bits */
    uint8_t  obstacle_state;    /* 3 bits */
    uint16_t obstacle_cm;       /* 9 bits, 0..511, saturates */
    uint8_t  fault_mask;        /* 8 bits */
    /* Appended at the end so older decoders still read the fields above. */
    int16_t  yaw_dps;           /* 9 bits, -256..255, sent as value + 256 */
    uint8_t  motion;            /* 3 bits, b4_motion_t */
    uint16_t up_speed_mm_s;     /* 8 bits, sent as value / 4: 0..1020 mm/s */
    uint8_t  tilt_deg;          /* 7 bits, 0..127, saturates */
    uint8_t  s5_fault;          /* 3 bits, subsystem5_fault_t */
} tlm_status_t;

/* ---------- Heartbeat frame (car -> dashboard, 1 Hz) ---------- */
typedef struct
{
    uint16_t session;
    uint8_t  seq;
    uint16_t uptime_s;          /* 16 bits, saturates at ~18 h */
    uint8_t  rssi_neg_dbm;      /* 7 bits, 0..127 meaning 0..-127 dBm */
    uint8_t  link_state;        /* 3 bits, telemetry_link_state_t */
    uint8_t  cmds_ok;           /* 8 bits, wraps */
    uint8_t  cmds_rejected;     /* 8 bits, wraps */
    uint8_t  queue_full;        /* 8 bits, wraps */
    uint8_t  reconnects;        /* 8 bits, wraps */
} tlm_heartbeat_t;

/* ---------- Event frame (car -> dashboard, on change, QoS 1) ---------- */
typedef enum
{
    TLM_EVT_STATE_CHANGE = 1,   /* a = old state, b = new state       */
    TLM_EVT_BARCODE      = 2,   /* a = barcode command (3 bits),
                                   b = letter code (6 bits: 0 none,
                                   1-26 A-Z, 27-36 0-9); a = NONE and
                                   b = 0: barcode seen, not decoded */
    TLM_EVT_HUMP         = 3,   /* a = height of a completed hump (mm) */
    TLM_EVT_OBSTACLE     = 4,   /* uses the obs_* fields              */
    TLM_EVT_LIFTED       = 5,   /* a = cause: 1 lifted, 2 overturned  */
    TLM_EVT_ESTOP        = 6    /* a = source                         */
} tlm_event_type_t;

typedef struct
{
    uint16_t session;
    uint8_t  seq;
    uint8_t  type;              /* 4 bits, tlm_event_type_t */
    uint8_t  a;
    uint8_t  b;
    uint8_t  scan_elems;        /* BARCODE: 6 bits, bars + spaces scanned */
    uint8_t  scan_result;       /* BARCODE: 2 bits, 0 ok 1 short 2 no '*'
                                   frame 3 bad letter */
    uint16_t obs_nearest_cm;    /* 9 bits */
    uint8_t  obs_centre_deg;    /* 8 bits, 0..180 */
    uint8_t  obs_width_cm;      /* 8 bits */
    uint16_t obs_clear_l_cm;    /* 9 bits */
    uint16_t obs_clear_r_cm;    /* 9 bits */
    uint8_t  obs_decision;      /* 3 bits */
} tlm_event_t;

/* ---------- Encoders: return bytes written, 0 if buffer too small ---------- */
uint32_t tlm_encode_online(uint16_t session, bool b_online,
                         uint8_t *p_buf, uint32_t cap);
uint32_t tlm_encode_status(const tlm_status_t *p_s, uint8_t *p_buf, uint32_t cap);
uint32_t tlm_encode_heartbeat(const tlm_heartbeat_t *p_h,
                            uint8_t *p_buf, uint32_t cap);
uint32_t tlm_encode_ack(uint16_t session, uint8_t seq, uint16_t cmd_seq,
                      uint8_t cmd_code, uint8_t result,
                      uint8_t *p_buf, uint32_t cap);
uint32_t tlm_encode_event(const tlm_event_t *p_e, uint8_t *p_buf, uint32_t cap);

/* Human-readable baseline (for debugging and the Week 13 size comparison). */
uint32_t tlm_format_status_json(const tlm_status_t *p_s, char *p_buf, uint32_t cap);

/* Command packing (dashboard side / tests) and validation (car side). */
uint32_t       tlm_encode_cmd(const tlm_cmd_t *p_cmd, uint8_t *p_buf, uint32_t cap);
tlm_result_t tlm_cmd_validate(const uint8_t *p_data, uint32_t len,
                              uint16_t current_session, uint16_t *p_last_seq,
                              tlm_cmd_t *p_out);

#endif /* TLM_CODEC_H */
