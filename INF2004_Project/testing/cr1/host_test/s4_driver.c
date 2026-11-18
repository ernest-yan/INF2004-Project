/**
 * CR1 S4 equivalence driver. run_host_tests.sh builds this TWICE: against
 * the pre-CR1 source tree (git archive pre-cr1) and the CR1 tree, and diffs
 * the outputs. The real imu.c, terrain.c, posture.c and motion_class.c are
 * compiled; only the I2C driver, the lock and the clock are stubbed.
 *
 * Simulated accelerometer runs (several seeds): calibrate still, drive,
 * cross a hump of random height, turn, then the IMU change-notice cases:
 * LIFTED (picked up) and, after an authorised reset, OVERTURNED. Every
 * event, every brake call and the status every 100 ms is printed.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* ---------- stubs ---------- */
#define APP_HW_H
typedef unsigned int UINT;
typedef uint32_t app_lock_t;
#define APP_LOCK_INIT(l) ((void)(l))
static int g_lock_depth;
static int g_lock_errors;
static inline UINT app_lock(app_lock_t *l)
{
    (void)l;
    if (++g_lock_depth > 1) { g_lock_errors++; }   /* spinlock is not recursive */
    return 0U;
}
static inline void app_unlock(app_lock_t *l, UINT s) { (void)l; (void)s; --g_lock_depth; }
static uint32_t g_now_us;
static inline uint32_t app_time_us_32(void) { return g_now_us; }
static inline uint64_t app_time_us_64(void) { return g_now_us; }

/* Accelerometer value the fake LSM303 returns (chip axes, mg). */
static int16_t g_ax, g_ay, g_az;
static int g_i2c_fail;
uint32_t drv_i2c1_init(unsigned int sda, unsigned int scl, uint32_t baud)
{ (void)sda; (void)scl; return baud; }
int drv_i2c1_write_timeout_us(uint8_t addr, const uint8_t *src, unsigned int len,
                              bool nostop, uint32_t t)
{ (void)addr; (void)src; (void)nostop; (void)t; return g_i2c_fail ? -1 : (int)len; }
int drv_i2c1_read_timeout_us(uint8_t addr, uint8_t *dst, unsigned int len,
                             bool nostop, uint32_t t)
{
    const int16_t v[3] = {g_ax, g_ay, g_az};
    (void)addr; (void)nostop; (void)t;
    if (g_i2c_fail) { return -1; }
    for (unsigned int i = 0U; (i < 3U) && ((2U * i + 1U) < len); i++)
    {
        const uint16_t raw = (uint16_t)(int16_t)(v[i] * 16);  /* 12-bit left-aligned */
        dst[2U * i] = (uint8_t)(raw & 0xFFU);
        dst[2U * i + 1U] = (uint8_t)(raw >> 8);
    }
    return (int)len;
}

/* The code under test. */
#include "imu.c"
#include "terrain.c"
#include "posture.c"
#include "motion_class.c"

/* ---------- driver ---------- */
static uint32_t g_rng;
static float frnd(void)          /* -1 .. 1 */
{
    g_rng = (g_rng * 1103515245U) + 12345U;
    return ((float)((g_rng >> 8) & 0xFFFFU) / 32768.0f) - 1.0f;
}

static int g_brakes;
static void on_brake(void *ctx) { (void)ctx; g_brakes++; }
static bool on_event(const b4_event_t *ev, void *ctx)
{
    (void)ctx;
    printf("  t=%7.2f EVENT type=%u height=%u max=%d min=%d gap=%u cause=%u fault=%u\n",
           (double)g_now_us / 1e6, (unsigned)ev->type, (unsigned)ev->height_mm,
           (int)ev->max_pitch_cdeg, (int)ev->min_pitch_cdeg,
           (unsigned)ev->peak_gap_mm, (unsigned)ev->cause, (unsigned)ev->fault);
    return true;
}

static void show(const char *tag)
{
    b4_status_t s;
    b4_get_status(&s);
#ifdef B4_FAULT_BIT
    const unsigned fault = s.fault_mask;
#else
    const unsigned fault = (s.fault == B4_FAULT_NONE) ? 0U : (1U << (s.fault - 1U));
#endif
    printf("  t=%7.2f %-8s st=%u flt=%u lift=%u cause=%u motion=%u cal=%u/%u "
           "pitch=%d tilt=%u up=%d yaw=%d humps=%u max=%u last=%u phase=%u brakes=%d\n",
           (double)g_now_us / 1e6, tag, (unsigned)s.state, fault,
           (unsigned)s.lift_latched, (unsigned)s.lift_cause, (unsigned)s.motion,
           (unsigned)s.calibration_ready, (unsigned)s.calibration_progress,
           (int)s.pitch_cdeg, (unsigned)s.tilt_cdeg, (int)s.recent_up_speed_mm_s,
           (int)s.yaw_rate_dps, (unsigned)s.hump_count,
           (unsigned)s.max_hump_height_mm, (unsigned)s.last_hump_height_mm,
           (unsigned)s.terrain_phase, g_brakes);
}

/* Car-frame acceleration (mg) -> chip axes: forward = chip Y, side = chip X. */
static void set_car(float fwd, float side, float up)
{
    g_ay = (int16_t)fwd;
    g_ax = (int16_t)side;
    g_az = (int16_t)up;
}

static uint32_t g_travel;
static float g_travel_f;
static void run(uint32_t ms, float pitch_deg, int32_t l, int32_t r,
                float extra_up_mg, float tilt_side_deg, float noise_mg)
{
    for (uint32_t t = 0U; t < ms; t += 10U)
    {
        const float p = pitch_deg * 0.0174533f;
        const float q = tilt_side_deg * 0.0174533f;
        g_now_us += 10000U;
        g_travel_f += (float)((l + r) / 2) * 0.01f;
        if (g_travel_f > (float)g_travel) { g_travel = (uint32_t)g_travel_f; }
        b4_update_travel_mm(g_travel);
        b4_update_wheel_speeds(l, r);
        set_car(-1000.0f * b4_sinf(p) + noise_mg * frnd(),
                1000.0f * b4_sinf(q) + noise_mg * frnd(),
                (1000.0f + extra_up_mg) * b4_sinf(p + 1.5707963f) * b4_sinf(q + 1.5707963f) + noise_mg * frnd());
        b4_poll();
        if ((g_now_us % 100000U) == 0U) { show("status"); }
    }
}

int main(void)
{
    static const b4_config_t cfg = {
        .i2c_addr = 0x19U, .i2c_speed_khz = 100U, .lift_accel_mg = 250U,
        .lift_speed_mm_s = 150U, .lift_tilt_deg = 30U, .overturn_tilt_deg = 60U,
        .wheel_track_mm = 120U, .wheelbase_mm = 130U, .forward_axis_sign = 1U,
        .forward_axis_is_y = 1U, .calibration_samples = 50U
    };
    for (uint32_t seed = 1U; seed <= 12U; seed++)
    {
        const float hump_deg = 2.5f + (float)(seed % 6U) * 1.5f;
        g_rng = seed * 7919U;
        g_now_us = 1000000U * seed;
        g_travel = 0U;
        g_travel_f = 0.0f;
        g_brakes = 0;
        printf("run %u (hump %.1f deg)\n", (unsigned)seed, (double)hump_deg);
        if (!b4_init(&cfg, on_event, on_brake, NULL)) { printf("  init failed\n"); }
        run(1000U, 0.0f, 0, 0, 0.0f, 0.0f, 3.0f);                 /* calibrate */
        show("calib");
        run(2000U, 0.0f, 100, 100, 0.0f, 0.0f, 15.0f);            /* drive */
        run(1000U, hump_deg, 100, 100, 0.0f, 0.0f, 15.0f);        /* hump up */
        run(1000U, -hump_deg, 100, 100, 0.0f, 0.0f, 15.0f);       /* hump down */
        run(2000U, 0.0f, 100, 100, 0.0f, 0.0f, 15.0f);            /* level */
        run(1500U, 0.0f, -60, 60, 0.0f, 0.0f, 15.0f);             /* pivot left */
        run(1000U, 0.0f, 0, 0, 0.0f, 0.0f, 5.0f);                 /* stop */
        show("before");
        run(300U, 0.0f, 0, 0, 450.0f, 0.0f, 20.0f);               /* picked up */
        run(1500U, 40.0f, 0, 0, 0.0f, 0.0f, 20.0f);               /* held tilted */
        show("lifted");
        run(1000U, 0.0f, 0, 0, 0.0f, 0.0f, 3.0f);                 /* put down */
        printf("  reset=%d\n", (int)b4_authorized_reset_while_stopped());
        run(1000U, 0.0f, 0, 0, 0.0f, 0.0f, 3.0f);                 /* recalibrate */
        run(1000U, 0.0f, 80, 80, 0.0f, 0.0f, 15.0f);
        run(2000U, 0.0f, 0, 0, 0.0f, 180.0f, 10.0f);              /* overturned */
        show("over");
        if ((seed % 4U) == 0U)                                    /* sensor fault */
        {
            g_i2c_fail = 1;
            run(100U, 0.0f, 0, 0, 0.0f, 0.0f, 0.0f);
            g_i2c_fail = 0;
            show("i2cfail");
        }
    }
    printf("lock errors (nested or unbalanced): %d / depth %d\n",
           g_lock_errors, g_lock_depth);
    return 0;
}
