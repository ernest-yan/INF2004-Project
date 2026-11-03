#ifndef B4_IMU_H
#define B4_IMU_H
#include <stdbool.h>
#include <stdint.h>
#include "terrain.h"
#define B4_I2C_SDA_GPIO       (26U)
#define B4_I2C_SCL_GPIO       (27U)
/* LSM303DLHC accelerometer address: SA0 high 0x19, low 0x18. */
#define B4_ACCEL_ADDR_DEFAULT (0x19U)
typedef enum
{
    B4_STATE_CALIBRATE = 0, B4_STATE_MONITOR,
    B4_STATE_IMPACT, B4_STATE_FAULT
} b4_state_t;
typedef enum
{
    B4_FAULT_NONE = 0, B4_FAULT_I2C, B4_FAULT_BAD_CONFIG,
    B4_FAULT_QUEUE_FULL
} b4_fault_t;
typedef enum
{
    B4_EVENT_HUMP_COMPLETE = 0, B4_EVENT_IMPACT,
    B4_EVENT_SENSOR_FAULT
} b4_event_type_t;
typedef struct
{
    uint8_t i2c_addr;              /* usually 0x19; verify actual module. */
    uint16_t i2c_speed_khz;        /* proposed 100/400. */
    uint16_t impact_threshold_mg;  /* provisional: e.g. 600 mg above 1g. */
    uint8_t forward_axis_sign;     /* 1 normal, 0 invert. Physical calibration. */
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
    uint16_t max_hump_height_mm;   /* rough calibrated estimate. */
    uint16_t last_hump_height_mm;
    uint16_t current_peak_mm;
    b4_terrain_phase_t terrain_phase;
    uint8_t impact_latched;
    uint32_t i2c_errors;
    uint64_t last_sample_us;
} b4_status_t;
typedef struct
{
    b4_event_type_t type;
    uint16_t height_mm;
    uint16_t acceleration_deviation_mg;
    b4_fault_t fault;
} b4_event_t;
typedef bool (*b4_event_publish_fn)(const b4_event_t *ev, void *ctx);
/* This is an urgent NOTIFICATION to VehicleController, not motor GPIO access. */
typedef void (*b4_urgent_brake_fn)(void *ctx);
bool b4_init(const b4_config_t *cfg, b4_event_publish_fn publish,
             b4_urgent_brake_fn urgent, void *ctx);
/* Task-only: call at ~10 ms for better impact sampling (still not guaranteed). */
void b4_poll(void);
/* Supply monotonic odometry (total forward path, mm) from Buddy 2. */
void b4_update_travel_mm(uint32_t cumulative_mm);
void b4_get_status(b4_status_t *out);
/* Only after authorised reset and sensor/vehicle safety checks. */
bool b4_authorized_reset_while_stopped(void);
#endif
