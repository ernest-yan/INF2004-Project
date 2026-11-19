/**
 * @file    team_interfaces.h
 * @brief   What Telemetry (S1) needs from the other subsystems.
 *
 * Telemetry only CALLS these. They are implemented in OUR folder by
 * team_adapters.c, which reads each buddy's real public getter
 * (b2_get_status, get_line_barcode_data_safe, b4_get_status,
 * subsystem5_get_status) and converts it. Call them only from our
 * telemetry task: some of those getters lock a mutex.
 *
 * Plain C types only (stdint/stdbool): safe next to tk/tkernel.h.
 */
#ifndef TEAM_INTERFACES_H
#define TEAM_INTERFACES_H

#include <stdbool.h>
#include <stdint.h>

/* ---------- VehicleController (owner: to be confirmed) ---------- */
typedef enum
{
    VEH_STATE_INIT = 0, VEH_STATE_IDLE, VEH_STATE_LINE_FOLLOW,
    VEH_STATE_LINE_SEARCH, VEH_STATE_BARCODE_NAV, VEH_STATE_OBSTACLE_AVOID,
    VEH_STATE_SAFE_STOP
} vehicle_state_t;                       /* 7 values -> 3 bits */

typedef enum
{
    VEH_EVT_NONE = 0, VEH_EVT_START, VEH_EVT_STOP
} vehicle_event_type_t;

typedef enum
{
    EVT_SRC_BUTTON = 0, EVT_SRC_REMOTE, EVT_SRC_IMU, EVT_SRC_MOTION,
    EVT_SRC_ULTRASONIC, EVT_SRC_LINE
} event_source_t;

typedef struct
{
    uint8_t  type;       /* vehicle_event_type_t */
    uint8_t  source;     /* event_source_t */
    uint16_t arg;        /* e.g. remote command seq, for logs */
    uint32_t t_rx_us;    /* time (us) when the request was received */
} vehicle_event_t;       /* 8 bytes -> qVehicleEvent message size */

/* CR1: the team structs below are filled and read only by our telemetry
 * task (single writer), and sized to their MQTT field widths. */
typedef struct
{
    uint8_t state : 3;            /* vehicle_state_t, 7 values */
    uint8_t b_estop_latched : 1;
    uint8_t estop_source : 3;     /* event_source_t of the latched E-STOP */
    uint8_t : 1;
    uint8_t fault_mask;           /* TLM_FAULT_Sn: bit n = subsystem n has a fault */
} vehicle_status_t;               /* CR1: 4 -> 2 bytes */

#define TLM_FAULT_S2    (1u << 2)
#define TLM_FAULT_S3    (1u << 3)
#define TLM_FAULT_S4    (1u << 4)
#define TLM_FAULT_S5    (1u << 5)

/* CR1: SYSTEM FAULT BITMASK -- one bit per fault condition, all subsystems
 * (testing/cr1/DATA_REPRESENTATION.md). Built on demand from each
 * subsystem's own fault bitmask, so it has no writer of its own and needs
 * no lock. The MQTT fault_mask byte (TLM_FAULT_Sn) is derived from it. */
#define SYS_FLT_MOTOR_STALL     (1u << 0)   /* S2 encoder stall */
#define SYS_FLT_GOAL_TIMEOUT    (1u << 1)   /* S2 */
#define SYS_FLT_S2_CONFIG       (1u << 2)   /* S2 */
#define SYS_FLT_S2_EVENT_LOSS   (1u << 3)   /* S2 */
#define SYS_FLT_LINE_LOSS       (1u << 4)   /* S3 (a state, not a fault) */
#define SYS_FLT_IMU_I2C         (1u << 5)   /* S4 */
#define SYS_FLT_IMU_CONFIG      (1u << 6)   /* S4 */
#define SYS_FLT_IMU_QUEUE       (1u << 7)   /* S4 */
#define SYS_FLT_NO_ECHO         (1u << 8)   /* S5 */
#define SYS_FLT_SCAN_INVALID    (1u << 9)   /* S5 */
#define SYS_FLT_STOP_TIMEOUT    (1u << 10)  /* S5 */
#define SYS_FLT_S5_EVENT        (1u << 11)  /* S5 */
#define SYS_FLT_NO_SAFE_SIDE    (1u << 12)  /* S5 */
#define SYS_FLT_BYPASS          (1u << 13)  /* S5 */
#define SYS_FLT_WIFI_LOSS       (1u << 14)  /* S1: MQTT not connected */
#define SYS_FLT_S2_ALL          (0x000Fu)
#define SYS_FLT_S4_ALL          (0x00E0u)
#define SYS_FLT_S5_ALL          (0x3F00u)

uint16_t system_fault_mask(void);

/* Remote START / STOP: set / clear s1_mission_go() (STOP also b2_stop()).
 * START is refused (false) while the car is in SAFE_STOP. */
bool vehicle_post_event(const vehicle_event_t *p_evt);

/* Remote E-STOP: b2_emergency_brake(), latched until a RESET. */
void safety_raise_estop(uint8_t source, uint32_t t_rx_us);

/* Remote RESET: clears S2's brake, S4's lift latch and S5's fault, only if the
 * wheels are still. Never re-enables driving (needs a new START).
 * Returns false if refused. */
bool safety_reset_while_stopped(void);

/* Remote barcode mapping -> S3's set_mqtt_action_mapping().
 * p_map = forward, U-turn, left, right (already validated). */
void barcode_set_mapping(const char *p_map);

void vehicle_get_status(vehicle_status_t *p_out);

/* ---------- MotionControl (Buddy 2) ---------- */
typedef struct
{
    uint16_t enc_l;               /* 16 bits, wraps (as on the wire) */
    uint16_t enc_r;
    uint32_t speed_l_mm_s : 10;   /* 0..1023, saturates */
    uint32_t speed_r_mm_s : 10;
    uint32_t distance_cm : 12;    /* 0..4095, saturates */
} motion_status_t;                /* CR1: 16 -> 8 bytes */

void motion_get_status(motion_status_t *p_out);

/* ---------- LineBarcode (Buddy 3) ----------
 * Buddy 3 uses digital IR sensors: left + centre follow the line, right
 * reads barcodes. Their getter is get_line_barcode_data_safe(), so our
 * team_adapters.c provides line_get_status() from it.
 */
typedef enum
{
    LINE_LOST = 0,      /* neither sensor on the line          */
    LINE_CENTRE,        /* centre only: on track                */
    LINE_LEFT_ONLY,     /* left only: drifting off the line     */
    LINE_JUNCTION       /* both: cross junction                 */
} line_state_t;                          /* = (left << 1) | centre, 2 bits */

typedef enum
{
    BARCODE_NONE = 0, BARCODE_LEFT, BARCODE_RIGHT, BARCODE_STRAIGHT,
    BARCODE_UTURN, BARCODE_INVALID
} barcode_cmd_t;                         /* 6 values -> 3 bits */

typedef struct
{
    uint8_t line_state : 2;  /* line_state_t */
    uint8_t last_barcode : 3; /* barcode_cmd_t */
    uint8_t : 3;
    uint8_t barcode_count;   /* +1 per decoded barcode (event trigger), wraps */
    uint8_t letter_code : 6; /* the decoded letter itself (S3_CODE: 1-26 = A-Z,
                                27-36 = 0-9, 0 = none), sent in the event */
    uint8_t scan_fail_count : 2; /* +1 per barcode SEEN but not decoded, wraps */
    uint8_t scan_elems : 6;  /* newest scan: bars + spaces (full barcode 29) */
    uint8_t scan_result : 2; /* newest scan: S3_SCAN_OK/SHORT/NO_FRAME/BAD_LETTER */
} line_status_t;             /* CR1: 4 -> 3 bytes (was 2 before the letter);
                                4 bytes with the scan diagnosis (19 Nov) */

void line_get_status(line_status_t *p_out);

/* ---------- ImuTerrain (Buddy 4) ---------- */
typedef struct
{
    int16_t  yaw_dps;        /* turn rate, + = left (from wheel speeds) */
    uint16_t up_speed_mm_s : 10; /* lift signal: recent upward speed, 0 if
                                    down; 0..1023 (wire sends /4) */
    uint16_t lift_cause : 2; /* 0 none, 1 lifted, 2 overturned */
    uint16_t motion : 3;     /* b4_motion_t: 7 values */
    uint16_t : 1;
    int8_t   pitch_deg;
    uint8_t  hump_max_mm;    /* highest hump so far (FR5) */
    uint8_t  last_hump_mm;   /* height of the last completed hump */
    uint8_t  hump_count;     /* +1 per completed hump (event trigger), wraps */
    uint8_t  lift_count;     /* +1 per lift/overturn latch (event trigger) */
    uint8_t  tilt_deg;       /* lift signal: tilt in any direction, 0..180 */
} imu_status_t;              /* CR1: 18 -> 10 bytes */

void imu_get_status(imu_status_t *p_out);

/* ---------- UltrasonicScan (Buddy 5) ---------- */
typedef struct
{
    uint32_t nearest_cm : 9;  /* 0..511 (0x1FF = no echo) */
    uint32_t clear_l_cm : 9;  /* 0..400 */
    uint32_t clear_r_cm : 9;
    uint32_t : 5;
    uint8_t  obstacle_state : 3; /* MONITOR..REVERSE, 8 values */
    uint8_t  fault : 3;       /* subsystem5_fault_t, 7 values */
    uint8_t  decision : 2;    /* S5 candidate side: 0 none, 1 left, 2 right */
    uint8_t  profile_count;   /* +1 per measured profile (event trigger),
                                 also when no bypass follows; wraps */
    uint8_t  centre_deg;
    uint8_t  width_cm;
} ultra_status_t;             /* CR1: 14 -> 8 bytes */

void ultra_get_status(ultra_status_t *p_out);

#endif /* TEAM_INTERFACES_H */
