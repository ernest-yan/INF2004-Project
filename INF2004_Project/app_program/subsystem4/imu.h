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
    uint8_t forward_axis_sign;     /* 1 normal, 0 invert. Physical calibration. */
    uint8_t forward_axis_is_y;     /* 0: chip X points forward, 1: chip Y does
                                      (GY-511 header mounted across the car). */
    uint8_t calibration_samples;   /* 20..100 samples, robot stationary. */
} b4_config_t;
typedef struct
{
    b4_state_t state;
    b4_fault_t fault;
    uint8_t calibration_progress;
    uint8_t calibration_ready;
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
    uint16_t max_hump_height_mm;   /* rough calibrated estimate. */
    uint16_t last_hump_height_mm;
    uint16_t current_peak_mm;
    uint16_t hump_count;           /* +1 per completed hump. */
    b4_terrain_phase_t terrain_phase;
    uint8_t lift_latched;          /* motors braked until authorised reset. */
    b4_lift_cause_t lift_cause;
    int16_t yaw_rate_dps;          /* from wheel speeds; + = turning left. */
    b4_motion_t motion;            /* debounced motion class. */
    uint32_t i2c_errors;
    uint32_t calibration_restarts; /* robot moved while calibrating. */
    uint64_t last_sample_us;
} b4_status_t;
typedef struct
{
    b4_event_type_t type;
    uint16_t height_mm;            /* HUMP_COMPLETE */
    b4_lift_cause_t cause;         /* LIFTED */
    b4_fault_t fault;              /* SENSOR_FAULT */
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
