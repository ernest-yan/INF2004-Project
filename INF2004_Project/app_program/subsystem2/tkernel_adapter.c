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

/* PROVISIONAL: calibrate on the assembled robot before enabling motion. */
#ifndef B2_CAL_TICKS_PER_WHEEL_REV
#define B2_CAL_TICKS_PER_WHEEL_REV       (20U)
#endif
#ifndef B2_CAL_WHEEL_CIRCUMFERENCE_MM
#define B2_CAL_WHEEL_CIRCUMFERENCE_MM   (210U)
#endif
#ifndef B2_CAL_WHEEL_TRACK_MM
#define B2_CAL_WHEEL_TRACK_MM           (120U)
#endif

static b2_config_t s2_config;
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
    s2_config.ticks_per_wheel_rev = B2_CAL_TICKS_PER_WHEEL_REV;
    s2_config.wheel_circumference_mm = B2_CAL_WHEEL_CIRCUMFERENCE_MM;
    s2_config.wheel_track_mm = B2_CAL_WHEEL_TRACK_MM;
    s2_config.max_speed_mm_s = 150U;
    s2_config.max_pwm_permil = 650U;
    s2_config.encoder_stall_ms = 500U;
    s2_config.pwm_frequency_hz = 20000U;
    s2_config.kp_q8 = 128U;
    s2_config.ki_q8 = 8U;
    return E_OK;
}

EXPORT ER subsystem2_start(void)
{
    /* Core 0 (TP_PRC1): the encoder IRQ is enabled on this core, and core 1
       is kept busy by the S3 barcode poller. */
    s2_task_id = app_start_task(subsystem2_task, S2_PRI_MAIN, TP_PRC1,
                                "motion_task");
    return (s2_task_id < E_OK) ? (ER)s2_task_id : E_OK;
}
