/*
 * Subsystem 2 integration glue: motion control task.
 *
 * The motion driver is hardware-specific. The constants below are starting
 * values only and MUST be replaced with measured wheel/encoder calibration.
 *
 * INTEGRATION FIX: b2_init() (pins, PWM, encoder interrupt) now runs INSIDE the
 * motion task, which is pinned to core 0.  The encoder interrupt is enabled in
 * the NVIC of the core that calls b2_init(), and TEAM.md requires that only the
 * owning task configures its peripherals.
 */
#include "tkernel_adapter.h"
#include "motion_cal.h" /* measured calibration: the one place it lives */

/* CR1 (R9): calibration is constant, so it lives in flash. b2_init keeps a
   pointer to it instead of a RAM copy. */
static const b2_config_t s2_config = {
    .ticks_per_rev_left = B2_CAL_TICKS_PER_REV_LEFT,
    .ticks_per_rev_right = B2_CAL_TICKS_PER_REV_RIGHT,
    .wheel_circumference_mm = B2_CAL_WHEEL_CIRCUMFERENCE_MM,
    .wheel_track_mm = B2_CAL_WHEEL_TRACK_MM,
    /* Also the feedforward scale: ff = max_pwm * request / max_speed.  At 150
       a 100 mm/s request started at ~225 mm/s (2026-11-13 telemetry, timer
       fixed), so steering pivots overshot the line.  300 halves it. */
    .max_speed_mm_s = 300U,
    .max_pwm_permil = 650U,
    .encoder_stall_ms = 500U,
    .pwm_frequency_hz = 20000U,
    /* Gains: defaults in tkernel_adapter.h, overridable from the build
       command (S2_KP / S2_KI / S2_KD / S2_SKP) for tuning runs. */
    .kp_q8 = B2_KP_Q8,
    .ki_q8 = B2_KI_Q8,
    /* D term: off until the tuning runs show it helps. The speed is a
       difference of encoder counts every 20 ms, so its derivative is noisy;
       S2_CAL=1 prints step responses to compare kd values. */
    .kd_q8 = B2_KD_Q8,
    /* Straight-line correction: 1 mm/s of trim per tick (~0.23 mm) that
       one wheel is ahead, at most 20 mm/s. Host-simulated with a 10 %
       weaker right motor (testing/cr1/host_test/test_s2_motion.c). */
    .straight_kp_q8 = B2_STRAIGHT_KP_Q8,
    .straight_trim_max_mm_s = 20U
};
static ID s2_task_id;
static volatile BOOL s2_hw_ready = FALSE;

EXPORT BOOL s2_ready(void)
{
    return s2_hw_ready;
}

static void subsystem2_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    if (!b2_init(&s2_config, NULL, NULL))
    {
        tm_printf((UB*)"ERROR S2: b2_init failed (bad config or IRQ define); "
                  "motion disabled\n");
        tk_ext_tsk();
    }
    s2_hw_ready = TRUE;
    tm_printf((UB*)"[S2] motion ready: PWM GP8-11, encoders GP6/GP7\n");

    for (;;)
    {
        b2_poll();
        (void)tk_dly_tsk(S2_POLL_PERIOD_MS);
    }
}

EXPORT ER subsystem2_init(void)
{
    return E_OK;    /* CR1: s2_config is a const initialiser now */
}

EXPORT ER subsystem2_start(void)
{
    /* Core 0 (TP_PRC1): the encoder IRQ is enabled on this core, and core 1
       is kept busy by the S3 barcode poller. */
    s2_task_id = app_start_task(subsystem2_task, S2_PRI_MAIN, TP_PRC1,
                                "motion_task");
    return (s2_task_id < E_OK) ? (ER)s2_task_id : E_OK;
}
