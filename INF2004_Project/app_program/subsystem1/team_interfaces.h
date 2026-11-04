/**
 * @file    team_interfaces.h
 * @brief   PROPOSED contract between Telemetry (S1) and the other subsystems.
 *
 * Telemetry only CALLS these. Each function is owned (implemented) by the
 * buddy named beside it, in THEIR subsystem folder. Until the real code
 * exists, team_stubs.c (in our folder) provides fake versions marked
 * "weak", so the real ones replace them automatically once they are added.
 * Agree names, fields and units with the team and record them in TEAM.md
 * section 5 (interfaces) before freezing.
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

typedef struct
{
    uint8_t state;       /* vehicle_state_t */
    bool    b_estop_latched;
    uint8_t estop_source; /* event_source_t of the latched E-STOP */
    uint8_t fault_mask;   /* one bit per subsystem fault */
} vehicle_status_t;

/* Non-blocking (tk_snd_mbf with TMO_POL). Returns false if qVehicleEvent
 * is full. Called from the WiFi service task, so it must never wait. */
bool vehicle_post_event(const vehicle_event_t *p_evt);

/* Urgent path, NOT through qVehicleEvent. Latches E-STOP and wakes the
 * brake path immediately (tk_set_flg). Never blocks. Safe from any task. */
void safety_raise_estop(uint8_t source, uint32_t t_rx_us);

void vehicle_get_status(vehicle_status_t *p_out);

/* ---------- MotionControl (Buddy 2) ---------- */
typedef struct
{
    uint16_t speed_l_mm_s;
    uint16_t speed_r_mm_s;
    uint32_t enc_l;
    uint32_t enc_r;
    uint32_t distance_mm;
} motion_status_t;

void motion_get_status(motion_status_t *p_out);

/* ---------- LineBarcode (Buddy 3) ----------
 * Buddy 3 uses digital IR sensors: left + centre follow the line, right
 * reads barcodes. Their getter is get_line_barcode_data_safe(), so our
 * s1_line_adapter.c (Stage 10) provides line_get_status() from it.
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
    uint8_t  line_state;     /* line_state_t */
    uint8_t  last_barcode;   /* barcode_cmd_t */
    uint16_t barcode_count;  /* +1 per decoded barcode (event trigger) */
} line_status_t;

void line_get_status(line_status_t *p_out);

/* ---------- ImuTerrain (Buddy 4) ---------- */
typedef struct
{
    int8_t   pitch_deg;
    uint8_t  hump_max_mm;    /* highest hump so far (FR5) */
    uint16_t impact_count;   /* +1 per detected impact (event trigger) */
} imu_status_t;

void imu_get_status(imu_status_t *p_out);

/* ---------- UltrasonicScan (Buddy 5) ---------- */
typedef struct
{
    uint8_t  obstacle_state;  /* MONITOR..FAULT_STOP, 8 values -> 3 bits */
    uint16_t nearest_cm;
    uint16_t profile_count;   /* +1 per completed profile (event trigger) */
    uint8_t  centre_deg;
    uint8_t  width_cm;
    uint16_t clear_l_cm;
    uint16_t clear_r_cm;
    uint8_t  decision;        /* stop/continue/left/right/reverse */
} ultra_status_t;

void ultra_get_status(ultra_status_t *p_out);

#endif /* TEAM_INTERFACES_H */
