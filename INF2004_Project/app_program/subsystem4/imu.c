/* IS33 Buddy 4. LSM303DLHC accelerometer; NO GYROSCOPE on this device.
 * Detects: lifted / overturned (-> brake, latched), hump height profile,
 * turn rate (from S2 wheel speeds) and a motion class.
 * Impact (collision) detection was removed on instruction: obstacle
 * avoidance means the robot must not collide in the first place. */
#include "imu.h"
#include "mathf.h"   /* PORT: replaces <math.h> (no libm linked) */
/* PORT: no Pico SDK in this firmware.  I2C goes through drv_i2c1 (I2C1 on
 * GP26/GP27), timing/locking through ../app_hw.h.  <stddef.h>/<string.h> are
 * not used (size_t clash with the kernel headers).  Changes marked PORT. */
#include "drv_i2c1.h"
#include "../app_hw.h"

/* PORT: lock replaces save_and_disable_interrupts(): b4_get_status() and
 * b4_update_travel_mm() may be called from a task on the other core. */
static app_lock_t s_lock;

#define B4_CTRL_REG1_A          (0x20U)
#define B4_CTRL_REG4_A          (0x23U)
#define B4_OUT_X_L_A            (0x28U)
#define B4_REGISTER_AUTOINC     (0x80U)
#define B4_CTRL1_100HZ_XYZ     (0x57U)
#define B4_CTRL4_BDU_HR_2G     (0x88U)
#define B4_I2C_TIMEOUT_US      (2000U)
#define B4_MAX_READING_MG      (2100.0f)
#define B4_PI                  (3.14159265f)
#define B4_MAX_TRAVEL_STEP_MM  (100U)
#define B4_CAL_SPREAD_MG       (100.0f) /* |a| range allowed while calibrating */
#define B4_MOTION_HOLD         (20U)    /* 200 ms before a new motion class */
static const b4_config_t *s_cfg;    /* CR1: points at the const config */
static b4_status_t s_status;
static b4_terrain_t s_terrain;
static b4_event_publish_fn s_publish;
static b4_urgent_brake_fn s_urgent;
static void *s_context;
static bool s_ready;
static float s_cal_pitch_sum;
static float s_cal_pitch_offset;
static float s_pitch_filtered;
static uint32_t s_travel_mm;
static uint32_t s_prev_travel_mm;
static uint8_t s_i2c_consecutive;
static float s_cal_x_sum, s_cal_y_sum, s_cal_z_sum;
static float s_cal_mag_min, s_cal_mag_max;
static b4_posture_t s_posture;
static int32_t s_left_mm_s, s_right_mm_s;
static b4_motion_t s_motion_candidate;
static uint8_t s_motion_count;
static uint8_t s_recent_age;

static int16_t b4_float_to_i16(float f)
{
    if (f > 32767.0f) { return 32767; }
    if (f < -32768.0f) { return -32768; }
    return (int16_t)f;
}
static bool b4_register_write(uint8_t reg, uint8_t value)
{
    const uint8_t data[2] = {reg, value};
    const int written = drv_i2c1_write_timeout_us(s_cfg->i2c_addr,
                                 data, sizeof(data), false, B4_I2C_TIMEOUT_US);
    return (written == (int)sizeof(data));
}
static bool b4_read_axes(int16_t *ax, int16_t *ay, int16_t *az)
{
    const uint8_t reg = B4_OUT_X_L_A | B4_REGISTER_AUTOINC;
    uint8_t raw[6];
    const int w = drv_i2c1_write_timeout_us(s_cfg->i2c_addr,
                                         &reg, 1U, true, B4_I2C_TIMEOUT_US);
    if (w != 1) { return false; }
    const int r = drv_i2c1_read_timeout_us(s_cfg->i2c_addr,
                             raw, sizeof(raw), false, B4_I2C_TIMEOUT_US);
    if (r != (int)sizeof(raw)) { return false; }
    /* High-resolution ±2g mode: 12-bit left-aligned, 1 mg/LSB nominal. */
    *ax = (int16_t)((int16_t)((uint16_t)raw[1] << 8U | raw[0]) / 16);
    *ay = (int16_t)((int16_t)((uint16_t)raw[3] << 8U | raw[2]) / 16);
    *az = (int16_t)((int16_t)((uint16_t)raw[5] << 8U | raw[4]) / 16);
    return true;
}
/* CR1: the packed control fields of s_status (state, fault_mask, lift_*,
 * motion, calibration_ready, terrain_phase) share storage and
 * b4_authorized_reset_while_stopped() writes them from another task, so
 * every write takes s_lock. Never call s_urgent / s_publish with it held. */
static void b4_set_fault(b4_fault_t fault)
{
    const UINT irq = app_lock(&s_lock);
    s_status.fault_mask |= B4_FAULT_BIT(fault);
    s_status.state = B4_STATE_FAULT;
    app_unlock(&s_lock, irq);
}
static void b4_publish(b4_event_t event)
{
    if ((s_publish != NULL) && !s_publish(&event, s_context))
    {
        b4_set_fault(B4_FAULT_QUEUE_FULL);
        if (s_urgent != NULL) { s_urgent(s_context); }
    }
}
/* Sensor fault: S4 can no longer see a lift, so stop the motors too. */
static void b4_fault(b4_fault_t fault)
{
    b4_set_fault(fault);
    if (s_urgent != NULL) { s_urgent(s_context); }
    const b4_event_t ev = {B4_EVENT_SENSOR_FAULT, 0U, 0, 0, 0U, B4_LIFT_NONE, fault};
    b4_publish(ev);
}
/* Call with s_lock held (calibration_ready is a packed control field). */
static void b4_calibration_restart_locked(void)
{
    s_status.calibration_progress = 0U;
    s_status.calibration_ready = 0U;
    s_cal_pitch_sum = 0.0f;
    s_cal_x_sum = 0.0f;
    s_cal_y_sum = 0.0f;
    s_cal_z_sum = 0.0f;
    s_cal_mag_min = 1.0e9f;
    s_cal_mag_max = 0.0f;
}
static void b4_set_motion(b4_motion_t m, bool immediate)
{
    if (immediate)
    {
        const UINT irq = app_lock(&s_lock);
        s_status.motion = (uint8_t)m;
        app_unlock(&s_lock, irq);
        s_motion_candidate = m;
        s_motion_count = B4_MOTION_HOLD;
        return;
    }
    if (m != s_motion_candidate)
    {
        s_motion_candidate = m;
        s_motion_count = 0U;
    }
    if (s_motion_count < B4_MOTION_HOLD)
    {
        s_motion_count++;
    }
    if ((s_motion_count >= B4_MOTION_HOLD) && (s_status.motion != (uint8_t)m))
    {
        const UINT irq = app_lock(&s_lock);
        s_status.motion = (uint8_t)m;
        app_unlock(&s_lock, irq);
    }
}
bool b4_init(const b4_config_t *cfg, b4_event_publish_fn publish,
             b4_urgent_brake_fn urgent, void *ctx)
{
    if ((cfg == NULL) ||
        ((cfg->i2c_addr != 0x18U) && (cfg->i2c_addr != 0x19U)) ||
        ((cfg->i2c_speed_khz != 100U) && (cfg->i2c_speed_khz != 400U)) ||
        (cfg->calibration_samples < 20U) || (cfg->calibration_samples > 100U) ||
        (cfg->lift_accel_mg < 100U) ||
        (cfg->lift_speed_mm_s < 50U) ||
        (cfg->lift_tilt_deg < 10U) ||
        (cfg->overturn_tilt_deg <= cfg->lift_tilt_deg) ||
        (cfg->overturn_tilt_deg > 170U) ||
        (cfg->wheel_track_mm == 0U) ||
        (cfg->wheelbase_mm < 50U) || (cfg->wheelbase_mm > 400U))
    {
        return false;
    }
    s_cfg = cfg;
    s_publish = publish;
    s_urgent = urgent;
    s_context = ctx;
    s_ready = false;
    {   /* PORT: struct copy replaces memset */
        static const b4_status_t zero_status;
        s_status = zero_status;
    }
    APP_LOCK_INIT(s_lock);
    {
        const UINT irq = app_lock(&s_lock);
        s_status.state = B4_STATE_CALIBRATE;
        s_status.motion = B4_MOTION_CALIBRATING;
        s_status.lift_cause = B4_LIFT_NONE;
        b4_calibration_restart_locked();
        app_unlock(&s_lock, irq);
    }
    b4_terrain_init(&s_terrain);
    s_cal_pitch_offset = 0.0f;
    s_pitch_filtered = 0.0f;
    s_travel_mm = 0U;
    s_prev_travel_mm = 0U;
    s_left_mm_s = 0;
    s_right_mm_s = 0;
    s_i2c_consecutive = 0U;
    s_motion_candidate = B4_MOTION_CALIBRATING;
    s_motion_count = 0U;
    /* PORT: replaces the SDK I2C init, pin function and pull-up calls */
    (void)drv_i2c1_init(B4_I2C_SDA_GPIO, B4_I2C_SCL_GPIO,
                        (uint32_t)cfg->i2c_speed_khz * 1000U);
    if (!b4_register_write(B4_CTRL_REG1_A, B4_CTRL1_100HZ_XYZ) ||
        !b4_register_write(B4_CTRL_REG4_A, B4_CTRL4_BDU_HR_2G))
    {
        b4_set_fault(B4_FAULT_I2C);
        return false;
    }
    s_ready = true;
    return true;
}
void b4_update_travel_mm(uint32_t cumulative_mm)
{
    const UINT irq = app_lock(&s_lock);
    if (cumulative_mm >= s_travel_mm)
    {
        s_travel_mm = cumulative_mm;
    }
    app_unlock(&s_lock, irq);
}
void b4_update_wheel_speeds(int32_t left_mm_s, int32_t right_mm_s)
{
    const UINT irq = app_lock(&s_lock);
    s_left_mm_s = left_mm_s;
    s_right_mm_s = right_mm_s;
    app_unlock(&s_lock, irq);
}
/* Calibration: the robot must be still. Collect the mean acceleration
   vector (= "up" for lift/overturn) and the mean pitch. If |a| varies by
   more than B4_CAL_SPREAD_MG the robot moved: start again. */
static void b4_calibrate_sample(float fx, float fy, float fz,
                                float magnitude, float measured_pitch)
{
    if (magnitude < s_cal_mag_min) { s_cal_mag_min = magnitude; }
    if (magnitude > s_cal_mag_max) { s_cal_mag_max = magnitude; }
    if ((s_cal_mag_max - s_cal_mag_min) > B4_CAL_SPREAD_MG)
    {
        const UINT irq = app_lock(&s_lock);
        if (s_status.calibration_restarts < UINT8_MAX)
        {
            s_status.calibration_restarts++;
        }
        b4_calibration_restart_locked();
        app_unlock(&s_lock, irq);
        return;
    }
    s_cal_pitch_sum += measured_pitch;
    s_cal_x_sum += fx;
    s_cal_y_sum += fy;
    s_cal_z_sum += fz;
    s_status.calibration_progress++;
    if (s_status.calibration_progress >= s_cfg->calibration_samples)
    {
        const float n = (float)s_cfg->calibration_samples;
        s_cal_pitch_offset = s_cal_pitch_sum / n;
        b4_posture_init(&s_posture, s_cal_x_sum / n, s_cal_y_sum / n,
                        s_cal_z_sum / n);
        s_pitch_filtered = 0.0f;
        s_status.peak_up_speed_mm_s = 0;
        s_status.peak_tilt_cdeg = 0U;
        {
            const UINT irq = app_lock(&s_lock);
            s_status.calibration_ready = 1U;
            s_status.state = B4_STATE_MONITOR;
            app_unlock(&s_lock, irq);
        }
        b4_set_motion(B4_MOTION_STATIONARY, true);
    }
}
void b4_poll(void)
{
    if ((!s_ready) || (s_status.state == B4_STATE_FAULT))
    {
        return;
    }
    int16_t ax, ay, az;
    if (!b4_read_axes(&ax, &ay, &az))
    {
        if (s_status.i2c_errors < UINT16_MAX) { s_status.i2c_errors++; }
        s_i2c_consecutive++;
        if (s_i2c_consecutive >= 3U) { b4_fault(B4_FAULT_I2C); }
        return;
    }
    s_i2c_consecutive = 0U;
    /* fx = car's forward axis, fy = car's sideways axis (chip X or Y). */
    const float fwd = (s_cfg->forward_axis_is_y != 0U) ? (float)ay : (float)ax;
    const float fx = (s_cfg->forward_axis_sign != 0U) ? fwd : -fwd;
    const float fy = (s_cfg->forward_axis_is_y != 0U) ? (float)ax : (float)ay;
    const float fz = (float)az;
    const float magnitude = b4_sqrtf((fx * fx) + (fy * fy) + (fz * fz));
    const float measured_pitch = b4_atan2f(-fx, b4_sqrtf(fy * fy + fz * fz)) *
                                 (180.0f / B4_PI);
    if ((magnitude > (B4_MAX_READING_MG + 100.0f)) ||
        (magnitude < 0.01f))
    {
        if (s_status.i2c_errors < UINT16_MAX) { s_status.i2c_errors++; }
        s_i2c_consecutive++;
        if (s_i2c_consecutive >= 3U) { b4_fault(B4_FAULT_I2C); }
        return;
    }
    s_status.acceleration_x_mg = ax;
    s_status.acceleration_y_mg = ay;
    s_status.acceleration_z_mg = az;
    s_status.last_sample_us = app_time_us_32();
    if (s_status.state == B4_STATE_CALIBRATE)
    {
        b4_calibrate_sample(fx, fy, fz, magnitude, measured_pitch);
        return;
    }
    const float relative_pitch = measured_pitch - s_cal_pitch_offset;
    s_pitch_filtered = s_pitch_filtered * 0.8f + relative_pitch * 0.2f;
    s_status.pitch_cdeg = b4_float_to_i16(s_pitch_filtered * 100.0f);

    /* ---- lifted / overturned ---------------------------------------- */
    const b4_posture_cfg_t pcfg = {s_cfg->lift_accel_mg,
                                   s_cfg->lift_speed_mm_s,
                                   s_cfg->lift_tilt_deg,
                                   s_cfg->overturn_tilt_deg};
    const b4_lift_cause_t cause = b4_posture_update(&s_posture, &pcfg,
                                                    fx, fy, fz);
    s_status.tilt_cdeg = (uint16_t)(s_posture.tilt_deg * 100.0f);
    {   /* peak-hold for 0.3 s so the 5 Hz telemetry cannot miss a spike */
        const int16_t up_now = b4_float_to_i16(s_posture.up_speed_mm_s);
        if ((up_now >= s_status.recent_up_speed_mm_s) || (s_recent_age >= 30U))
        {
            s_status.recent_up_speed_mm_s = up_now;
            s_recent_age = 0U;
        }
        else
        {
            s_recent_age++;
        }
    }
    if ((s_status.state == B4_STATE_MONITOR) && (cause == B4_LIFT_NONE))
    {
        const int16_t up = b4_float_to_i16(s_posture.up_speed_mm_s);
        if (up > s_status.peak_up_speed_mm_s)
        {
            s_status.peak_up_speed_mm_s = up;
        }
        if (s_status.tilt_cdeg > s_status.peak_tilt_cdeg)
        {
            s_status.peak_tilt_cdeg = s_status.tilt_cdeg;
        }
    }
    if (s_status.state == B4_STATE_LIFTED)
    {
        /* Already braked. A lift that ends upside down is reported as
           OVERTURNED (the more specific cause); never downgraded. */
        if ((cause == B4_LIFT_OVERTURNED) &&
            (s_status.lift_cause != B4_LIFT_OVERTURNED))
        {
            const UINT irq = app_lock(&s_lock);
            s_status.lift_cause = B4_LIFT_OVERTURNED;
            app_unlock(&s_lock, irq);
            b4_set_motion(B4_MOTION_OVERTURNED, true);
        }
        return;
    }
    if (cause != B4_LIFT_NONE)
    {
        if (s_urgent != NULL) { s_urgent(s_context); }  /* brake first */
        {
            const UINT irq = app_lock(&s_lock);
            s_status.state = B4_STATE_LIFTED;
            s_status.lift_latched = 1U;
            s_status.lift_cause = (uint8_t)cause;
            app_unlock(&s_lock, irq);
        }
        b4_set_motion((cause == B4_LIFT_OVERTURNED) ? B4_MOTION_OVERTURNED
                                                    : B4_MOTION_LIFTED, true);
        const b4_event_t ev = {B4_EVENT_LIFTED, 0U, 0, 0, 0U, cause, B4_FAULT_NONE};
        b4_publish(ev);
        return;
    }

    /* ---- hump profile ------------------------------------------------ */
    uint32_t travel_now;
    int32_t left_mm_s, right_mm_s;
    UINT irq = app_lock(&s_lock);
    travel_now = s_travel_mm;
    left_mm_s = s_left_mm_s;
    right_mm_s = s_right_mm_s;
    app_unlock(&s_lock, irq);
    const uint32_t step_mm = travel_now - s_prev_travel_mm;
    s_prev_travel_mm = travel_now;
    const uint32_t safe_step = (step_mm <= B4_MAX_TRAVEL_STEP_MM) ? step_mm : 0U;
    if (b4_terrain_update(&s_terrain, s_pitch_filtered, safe_step,
                          s_cfg->wheelbase_mm))
    {
        s_status.hump_count++;          /* wraps: readers compare for change */
        const b4_event_t ev = {B4_EVENT_HUMP_COMPLETE,
            s_terrain.last_peak_mm, s_terrain.last_max_cdeg,
            s_terrain.last_min_cdeg, s_terrain.last_gap_mm,
            B4_LIFT_NONE, B4_FAULT_NONE};
        b4_publish(ev);
    }
    s_status.last_hump_height_mm = s_terrain.last_peak_mm;
    s_status.current_peak_mm = s_terrain.peak_mm;
    s_status.max_hump_height_mm = s_terrain.max_peak_mm;
    irq = app_lock(&s_lock);
    s_status.terrain_phase = (uint8_t)s_terrain.phase;
    app_unlock(&s_lock, irq);

    /* ---- turn rate and motion class ---------------------------------- */
    const float yaw = b4_yaw_rate_dps(left_mm_s, right_mm_s,
                                      s_cfg->wheel_track_mm);
    s_status.yaw_rate_dps = b4_float_to_i16(yaw);
    b4_set_motion(b4_classify_motion(true, B4_LIFT_NONE, s_terrain.phase,
                                     left_mm_s, right_mm_s, yaw), false);
}
void b4_get_status(b4_status_t *out)
{
    if (out != NULL)
    {
        const UINT irq = app_lock(&s_lock);
        *out = s_status;
        app_unlock(&s_lock, irq);
    }
}
bool b4_authorized_reset_while_stopped(void)
{
    if (!s_ready) { return false; }
    const UINT irq = app_lock(&s_lock);
    s_status.fault_mask = 0U;
    s_status.state = B4_STATE_CALIBRATE;
    s_status.lift_latched = 0U;
    s_status.lift_cause = B4_LIFT_NONE;
    s_status.motion = B4_MOTION_CALIBRATING;
    s_motion_candidate = B4_MOTION_CALIBRATING;
    s_motion_count = 0U;
    b4_calibration_restart_locked();
    s_i2c_consecutive = 0U;
    s_prev_travel_mm = s_travel_mm;
    /* Preserve max hump encountered during the existing mission. */
    const uint16_t saved_max = s_terrain.max_peak_mm;
    b4_terrain_init(&s_terrain);
    s_terrain.max_peak_mm = saved_max;
    app_unlock(&s_lock, irq);
    return true;
}
