/* IS33 Buddy 4. LSM303DLHC accelerometer; NO GYROSCOPE on this device. */
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
#define B4_GRAVITY_MG          (1000.0f)
#define B4_MAX_READING_MG      (2100.0f)
#define B4_PI                  (3.14159265f)
#define B4_MAX_TRAVEL_STEP_MM  (100U)
static b4_config_t s_cfg;
static b4_status_t s_status;
static b4_terrain_t s_terrain;
static b4_event_publish_fn s_publish;
static b4_urgent_brake_fn s_urgent;
static void *s_context;
static bool s_ready;
static float s_cal_pitch_sum;
static float s_cal_pitch_offset;
static float s_cal_magnitude_sum;
static float s_magnitude_offset;
static float s_pitch_filtered;
static uint32_t s_travel_mm;
static uint32_t s_prev_travel_mm;
static uint8_t s_i2c_consecutive;
static uint8_t s_impact_consecutive;

static int16_t b4_float_to_i16(float f)
{
    if (f > 32767.0f) { return 32767; }
    if (f < -32768.0f) { return -32768; }
    return (int16_t)f;
}
static uint16_t b4_float_to_u16(float f)
{
    if (f <= 0.0f) { return 0U; }
    if (f > 65535.0f) { return 65535U; }
    return (uint16_t)f;
}
static bool b4_register_write(uint8_t reg, uint8_t value)
{
    const uint8_t data[2] = {reg, value};
    const int written = drv_i2c1_write_timeout_us(s_cfg.i2c_addr,
                                 data, sizeof(data), false, B4_I2C_TIMEOUT_US);
    return (written == (int)sizeof(data));
}
static bool b4_read_axes(int16_t *ax, int16_t *ay, int16_t *az)
{
    const uint8_t reg = B4_OUT_X_L_A | B4_REGISTER_AUTOINC;
    uint8_t raw[6];
    const int w = drv_i2c1_write_timeout_us(s_cfg.i2c_addr,
                                         &reg, 1U, true, B4_I2C_TIMEOUT_US);
    if (w != 1) { return false; }
    const int r = drv_i2c1_read_timeout_us(s_cfg.i2c_addr,
                             raw, sizeof(raw), false, B4_I2C_TIMEOUT_US);
    if (r != (int)sizeof(raw)) { return false; }
    /* High-resolution ±2g mode: 12-bit left-aligned, 1 mg/LSB nominal. */
    *ax = (int16_t)((int16_t)((uint16_t)raw[1] << 8U | raw[0]) / 16);
    *ay = (int16_t)((int16_t)((uint16_t)raw[3] << 8U | raw[2]) / 16);
    *az = (int16_t)((int16_t)((uint16_t)raw[5] << 8U | raw[4]) / 16);
    return true;
}
static void b4_publish(b4_event_t event)
{
    if ((s_publish != NULL) && !s_publish(&event, s_context))
    {
        s_status.fault = B4_FAULT_QUEUE_FULL;
        s_status.state = B4_STATE_FAULT;
        s_status.impact_latched = 1U;
        if (s_urgent != NULL) { s_urgent(s_context); }
    }
}
static void b4_fault(b4_fault_t fault)
{
    s_status.state = B4_STATE_FAULT;
    s_status.fault = fault;
    s_status.impact_latched = 1U;
    if (s_urgent != NULL) { s_urgent(s_context); }
    const b4_event_t ev = {B4_EVENT_SENSOR_FAULT, 0U, 0U, fault};
    b4_publish(ev);
}
bool b4_init(const b4_config_t *cfg, b4_event_publish_fn publish,
             b4_urgent_brake_fn urgent, void *ctx)
{
    if ((cfg == NULL) ||
        ((cfg->i2c_addr != 0x18U) && (cfg->i2c_addr != 0x19U)) ||
        ((cfg->i2c_speed_khz != 100U) && (cfg->i2c_speed_khz != 400U)) ||
        (cfg->calibration_samples < 20U) || (cfg->calibration_samples > 100U) ||
        (cfg->impact_threshold_mg < 200U))
    {
        return false;
    }
    s_cfg = *cfg;
    s_publish = publish;
    s_urgent = urgent;
    s_context = ctx;
    s_ready = false;
    {   /* PORT: struct copy replaces memset */
        static const b4_status_t zero_status;
        s_status = zero_status;
    }
    APP_LOCK_INIT(s_lock);
    s_status.state = B4_STATE_CALIBRATE;
    b4_terrain_init(&s_terrain);
    s_cal_pitch_sum = 0.0f;
    s_cal_magnitude_sum = 0.0f;
    s_cal_pitch_offset = 0.0f;
    s_magnitude_offset = 1000.0f;
    s_pitch_filtered = 0.0f;
    s_travel_mm = 0U;
    s_prev_travel_mm = 0U;
    s_i2c_consecutive = 0U;
    s_impact_consecutive = 0U;
    /* PORT: replaces the SDK I2C init, pin function and pull-up calls */
    (void)drv_i2c1_init(B4_I2C_SDA_GPIO, B4_I2C_SCL_GPIO,
                        (uint32_t)cfg->i2c_speed_khz * 1000U);
    if (!b4_register_write(B4_CTRL_REG1_A, B4_CTRL1_100HZ_XYZ) ||
        !b4_register_write(B4_CTRL_REG4_A, B4_CTRL4_BDU_HR_2G))
    {
        s_status.state = B4_STATE_FAULT;
        s_status.fault = B4_FAULT_I2C;
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
void b4_poll(void)
{
    if ((!s_ready) || (s_status.state == B4_STATE_FAULT) ||
        (s_status.state == B4_STATE_IMPACT))
    {
        return;
    }
    int16_t ax, ay, az;
    if (!b4_read_axes(&ax, &ay, &az))
    {
        s_status.i2c_errors++;
        s_i2c_consecutive++;
        if (s_i2c_consecutive >= 3U) { b4_fault(B4_FAULT_I2C); }
        return;
    }
    s_i2c_consecutive = 0U;
    const float fx = (s_cfg.forward_axis_sign != 0U) ? (float)ax : -(float)ax;
    const float fy = (float)ay;
    const float fz = (float)az;
    const float magnitude = b4_sqrtf((fx * fx) + (fy * fy) + (fz * fz));
    const float measured_pitch = b4_atan2f(-fx, b4_sqrtf(fy * fy + fz * fz)) *
                                 (180.0f / B4_PI);
    if ((magnitude > (B4_MAX_READING_MG + 100.0f)) ||
        (magnitude < 0.01f))
    {
        s_status.i2c_errors++;
        s_i2c_consecutive++;
        if (s_i2c_consecutive >= 3U) { b4_fault(B4_FAULT_I2C); }
        return;
    }
    s_status.acceleration_x_mg = ax;
    s_status.acceleration_y_mg = ay;
    s_status.acceleration_z_mg = az;
    s_status.last_sample_us = app_time_us_64();
    if (s_status.state == B4_STATE_CALIBRATE)
    {
        s_cal_pitch_sum += measured_pitch;
        s_cal_magnitude_sum += magnitude;
        s_status.calibration_progress++;
        if (s_status.calibration_progress >= s_cfg.calibration_samples)
        {
            s_cal_pitch_offset = s_cal_pitch_sum /
                                  (float)s_cfg.calibration_samples;
            s_magnitude_offset = s_cal_magnitude_sum /
                                  (float)s_cfg.calibration_samples;
            s_status.calibration_ready = 1U;
            s_status.state = B4_STATE_MONITOR;
        }
        return;
    }
    const float relative_pitch = measured_pitch - s_cal_pitch_offset;
    s_pitch_filtered = s_pitch_filtered * 0.8f + relative_pitch * 0.2f;
    s_status.pitch_cdeg = b4_float_to_i16(s_pitch_filtered * 100.0f);
    float deviation = b4_fabsf(magnitude - s_magnitude_offset);
    if (deviation > 65535.0f) { deviation = 65535.0f; }
    /* Single very large hit or two threshold crossings: provisional tuning. */
    if (deviation > ((float)s_cfg.impact_threshold_mg))
    {
        s_impact_consecutive++;
    }
    else
    {
        s_impact_consecutive = 0U;
    }
    if ((s_impact_consecutive >= 2U) ||
        (deviation > (2.0f * (float)s_cfg.impact_threshold_mg)))
    {
        s_status.state = B4_STATE_IMPACT;
        s_status.impact_latched = 1U;
        if (s_urgent != NULL) { s_urgent(s_context); }
        const b4_event_t ev = {B4_EVENT_IMPACT, 0U,
                                   b4_float_to_u16(deviation), B4_FAULT_NONE};
        b4_publish(ev);
        return;
    }
    uint32_t travel_now;
    const UINT irq = app_lock(&s_lock);
    travel_now = s_travel_mm;
    app_unlock(&s_lock, irq);
    const uint32_t step_mm = travel_now - s_prev_travel_mm;
    s_prev_travel_mm = travel_now;
    const uint32_t safe_step = (step_mm <= B4_MAX_TRAVEL_STEP_MM) ? step_mm : 0U;
    if (b4_terrain_update(&s_terrain, s_pitch_filtered, safe_step))
    {
        const b4_event_t ev = {B4_EVENT_HUMP_COMPLETE,
            s_terrain.last_peak_mm, 0U, B4_FAULT_NONE};
        b4_publish(ev);
    }
    s_status.last_hump_height_mm = s_terrain.last_peak_mm;
    s_status.current_peak_mm = s_terrain.peak_mm;
    s_status.max_hump_height_mm = s_terrain.max_peak_mm;
    s_status.terrain_phase = s_terrain.phase;
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
    s_status.fault = B4_FAULT_NONE;
    s_status.state = B4_STATE_CALIBRATE;
    s_status.impact_latched = 0U;
    s_status.calibration_ready = 0U;
    s_status.calibration_progress = 0U;
    s_cal_pitch_sum = 0.0f;
    s_cal_magnitude_sum = 0.0f;
    s_impact_consecutive = 0U;
    s_i2c_consecutive = 0U;
    s_prev_travel_mm = s_travel_mm;
    /* Preserve max hump encountered during the existing mission. */
    const uint16_t saved_max = s_terrain.max_peak_mm;
    b4_terrain_init(&s_terrain);
    s_terrain.max_peak_mm = saved_max;
    app_unlock(&s_lock, irq);
    return true;
}
