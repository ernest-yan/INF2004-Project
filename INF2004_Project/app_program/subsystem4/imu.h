/*
 * imu.h
 * Subsystem 4 -- LSM303DLHC accelerometer: calibration, tilt, hump height,
 * lift / overturn detection and motion class, polled at 100 Hz.
 */
#ifndef B4_IMU_H
#define B4_IMU_H
#include <stdbool.h>
#include <stdint.h>
#include "terrain.h"
#include "posture.h"
#include "motion_class.h"
#define B4_I2C_SDA_GPIO       (26U)
#define B4_I2C_SCL_GPIO       (27U)
/* LSM303DLHC accelerometer address: SA0 high 0x19, low 0x18. */
#define B4_ACCEL_ADDR_DEFAULT (0x19U)
typedef enum
{
    B4_STATE_CALIBRATE = 0, B4_STATE_MONITOR,
    B4_STATE_LIFTED,               /* lifted or overturned: latched */
    B4_STATE_FAULT
} b4_state_t;
typedef enum
{
    B4_FAULT_NONE = 0, B4_FAULT_I2C, B4_FAULT_BAD_CONFIG,
    B4_FAULT_QUEUE_FULL
} b4_fault_t;
typedef enum
{
    B4_EVENT_HUMP_COMPLETE = 0, B4_EVENT_LIFTED,
    B4_EVENT_SENSOR_FAULT
} b4_event_type_t;
typedef struct
{
    uint8_t i2c_addr;              /* usually 0x19; verify actual module. */
    uint16_t i2c_speed_khz;        /* proposed 100/400. */
    uint16_t lift_accel_mg;        /* PROVISIONAL: upward accel above 1 g. */
    uint16_t lift_speed_mm_s;      /* PROVISIONAL: upward speed = picked up. */
    uint8_t lift_tilt_deg;         /* PROVISIONAL: held tilt = picked up;
                                      must be above the steepest hump. */
    uint8_t overturn_tilt_deg;     /* PROVISIONAL: on its side or roof. */
    uint16_t wheel_track_mm;       /* = S2's B2_CAL_WHEEL_TRACK_MM. */
    uint16_t wheelbase_mm;         /* front-to-rear ground contacts (hump
                                      height = wheelbase x sin(peak tilt)). */
    uint8_t forward_axis_sign;     /* 1 normal, 0 invert. Physical calibration. */
    uint8_t forward_axis_is_y;     /* 0: chip X points forward, 1: chip Y does
                                      (GY-511 header mounted across the car). */
    uint8_t calibration_samples;   /* 20..100 samples, robot stationary. */
} b4_config_t;
/* CR1: fault bitmask, bit (f - 1) for each b4_fault_t f != NONE. */
#define B4_FAULT_BIT(f)  ((uint8_t)(1U << ((uint32_t)(f) - 1U)))

/* CR1: 56 -> 32 bytes.
 * The packed control fields at the end share storage and are also written
 * by b4_authorized_reset_while_stopped() on another task, so EVERY write to
 * them holds s_lock (imu.c). Values updated on every 100 Hz sample outside
 * the lock stay separate plain fields (no shared read-modify-write). */
typedef struct
{
    uint32_t last_sample_us;       /* 32-bit us, wraps every 71.6 min. */
    uint16_t i2c_errors;           /* saturates. */
    int16_t acceleration_x_mg;
    int16_t acceleration_y_mg;
    int16_t acceleration_z_mg;
    int16_t pitch_cdeg;            /* degrees * 100, relative to startup. */
    uint16_t tilt_cdeg;            /* any direction, degrees * 100, 0..18000. */
    /* Tuning aid: highest values seen since calibration WITHOUT a lift.
       Compare with lift_speed_mm_s / lift_tilt_deg after a full run. */
    int16_t peak_up_speed_mm_s;
    uint16_t peak_tilt_cdeg;
    int16_t recent_up_speed_mm_s;  /* highest upward speed in the last 0.3 s
                                      (for 5 Hz telemetry), + = up. */
    int16_t yaw_rate_dps;          /* from wheel speeds; + = turning left. */
    uint8_t max_hump_height_mm;    /* rough calibrated estimate, 0..255. */
    uint8_t last_hump_height_mm;
    uint8_t current_peak_mm;
    uint8_t hump_count;            /* +1 per completed hump, wraps. */
    uint8_t calibration_progress;  /* 0..calibration_samples (<= 100) */
    uint8_t calibration_restarts;  /* robot moved while calibrating; saturates. */
    /* packed control fields: write only while holding s_lock */
    uint8_t state : 2;             /* b4_state_t, 4 values */
    uint8_t fault_mask : 3;        /* B4_FAULT_BIT() per active fault */
    uint8_t lift_latched : 1;      /* motors braked until authorised reset. */
    uint8_t lift_cause : 2;        /* b4_lift_cause_t, 3 values */
    uint8_t motion : 3;            /* b4_motion_t, 7 values (debounced). */
    uint8_t calibration_ready : 1;
    uint8_t terrain_phase : 2;     /* b4_terrain_phase_t, 4 values */
    uint8_t : 2;
} b4_status_t;
/* CR1: 12 -> 8 bytes. Field ORDER unchanged: imu.c builds events with
 * positional initialisers. */
typedef struct
{
    uint8_t type : 2;              /* b4_event_type_t, 3 values */
    uint8_t : 6;
    uint8_t height_mm;             /* HUMP_COMPLETE, 0..255 */
    int16_t max_pitch_cdeg;        /* HUMP_COMPLETE: peak nose-up */
    int16_t min_pitch_cdeg;        /* HUMP_COMPLETE: peak nose-down */
    uint16_t peak_gap_mm : 10;     /* HUMP_COMPLETE: travel between the two
                                      peaks (<= 700), about the wheelbase */
    uint16_t cause : 2;            /* LIFTED: b4_lift_cause_t */
    uint16_t fault : 3;            /* SENSOR_FAULT: b4_fault_t */
    uint16_t : 1;
} b4_event_t;
typedef bool (*b4_event_publish_fn)(const b4_event_t *ev, void *ctx);
/* This is an urgent NOTIFICATION to VehicleController, not motor GPIO access. */
typedef void (*b4_urgent_brake_fn)(void *ctx);
bool b4_init(const b4_config_t *cfg, b4_event_publish_fn publish,
             b4_urgent_brake_fn urgent, void *ctx);
/* Task-only: call every 10 ms (lift hold times assume 100 Hz). */
void b4_poll(void);
/* Supply monotonic odometry (total forward path, mm) from Buddy 2. */
void b4_update_travel_mm(uint32_t cumulative_mm);
/* Supply signed wheel speeds (mm/s) from Buddy 2, for turn rate. */
void b4_update_wheel_speeds(int32_t left_mm_s, int32_t right_mm_s);
void b4_get_status(b4_status_t *out);
/* Only after authorised reset and sensor/vehicle safety checks.
   The robot must be back on the ground and still: S4 recalibrates. */
bool b4_authorized_reset_while_stopped(void);
#endif
