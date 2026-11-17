/*
 * Subsystem 4 integration glue: LSM303DLHC accelerometer task.
 * GY-511 on I2C1: GP26 = SDA, GP27 = SCL (avoids S3's GP2/GP4 line sensors).
 *
 * INTEGRATION FIXES:
 *  - b4_init() (I2C1 + sensor set-up) now runs inside imu_task, the owner of
 *    I2C1, and is retried every S4_RETRY_MS if the sensor does not answer
 *    (instead of failing once at boot and staying dead).
 *  - imu_task feeds S2's encoder odometry into b4_update_travel_mm(); before
 *    this nothing called it, so the hump profile could never complete.
 *
 * LIFT / OVERTURN (replaces impact detection, per the updated brief):
 *  - lifting or overturning the robot calls b2_emergency_brake() (IF-04);
 *    the brake stays latched until a remote RESET, so lifting the robot at
 *    the end of the demo run stops it for good.
 *  - imu_task also feeds S2's wheel speeds into b4_update_wheel_speeds()
 *    (turn rate + motion class).
 *  - S4's events (hump, lift, sensor fault) are printed on the console;
 *    S1 reports them over MQTT from b4_get_status() (hump_count etc.).
 */
#include "tkernel_adapter.h"
#include "../subsystem2/tkernel_adapter.h"

static ID s4_task_id;
/* CR1 (R9): calibration is constant, so it lives in flash; b4_init keeps a
   pointer to it instead of a RAM copy. */
static const b4_config_t s4_config = {
    .i2c_addr = B4_ACCEL_ADDR_DEFAULT,
    .i2c_speed_khz = 100U,
    /* Lift / overturn: PROVISIONAL, tune on the robot (S4-M01..M04).
       lift_tilt_deg must stay above the steepest hump on the course. */
    .lift_accel_mg = 250U,
    /* A hand lift is 200-500 mm/s; a hump at line speed is ~20 mm/s. */
    .lift_speed_mm_s = 150U,
    .lift_tilt_deg = 30U,
    .overturn_tilt_deg = 60U,
    .wheel_track_mm = S4_WHEEL_TRACK_MM,
    .wheelbase_mm = S4_WHEELBASE_MM,
    /* GY-511 is mounted with its pin header across the car, so the chip's
       Y axis is the car's forward axis.  Nose-up must give positive pitch:
       if it reads negative, set forward_axis_sign = 0U. */
    .forward_axis_is_y = 1U,
    .forward_axis_sign = 1U,
    .calibration_samples = 50U
};
static volatile BOOL s4_hw_ready = FALSE;

EXPORT BOOL s4_ready(void)
{
    return s4_hw_ready;
}

static void subsystem4_urgent_brake(void *context)
{
    (void)context;
    b2_emergency_brake();
}

static const char *const s4_cause_name[] = {"-", "LIFTED", "OVERTURNED"};

/* S4 event sink: console log (bench evidence). Never fails, so S4 never
   treats a slow console as a lost event. */
static bool subsystem4_event(const b4_event_t *ev, void *context)
{
    (void)context;
    switch (ev->type) {
    case B4_EVENT_HUMP_COMPLETE:
        tm_printf((UB*)"[S4] hump complete: %u mm (peak tilt +%d.%02d / "
                  "-%d.%02d deg, peaks %u mm apart; wheelbase set to %u mm)\n",
                  (unsigned)ev->height_mm,
                  (int)(ev->max_pitch_cdeg / 100),
                  (int)(ev->max_pitch_cdeg % 100),
                  (int)(-ev->min_pitch_cdeg / 100),
                  (int)(-ev->min_pitch_cdeg % 100),
                  (unsigned)ev->peak_gap_mm,
                  (unsigned)s4_config.wheelbase_mm);
        break;
    case B4_EVENT_LIFTED:
        tm_printf((UB*)"[S4] %s - motors stopped (RESET to clear)\n",
                  (ev->cause <= B4_LIFT_OVERTURNED) ?
                  s4_cause_name[ev->cause] : "?");
        break;
    default:
        tm_printf((UB*)"[S4] sensor fault %d - motors stopped\n",
                  (int)ev->fault);
        break;
    }
    return true;
}

/* Forward path from S2 odometry: mean of both wheels, only ever increasing
   (b4_update_travel_mm ignores decreases, e.g. when reversing). */
static void subsystem4_feed_odometry(void)
{
    b2_status_t st;
    int32_t mean_mm;

    if (!s2_ready())
    {
        return;
    }
    b2_get_status(&st);
    mean_mm = (st.left_total_mm / 2) + (st.right_total_mm / 2);
    if (mean_mm > 0)
    {
        b4_update_travel_mm((uint32_t)mean_mm);
    }
    b4_update_wheel_speeds(b2_speed_value(st.left_speed_ob),
                           b2_speed_value(st.right_speed_ob));
}

/* Tuning aid: every 2 s, if a new highest upward speed or tilt was seen
   while NOT lifted, print it. After a full run these show how close
   normal driving comes to the lift thresholds. */
static void subsystem4_report_peaks(void)
{
    static UW polls;
    static int16_t shown_up;
    static uint16_t shown_tilt;
    b4_status_t st;

    if (++polls < (2000U / S4_SAMPLE_MS))
    {
        return;
    }
    polls = 0U;
    b4_get_status(&st);
    if ((st.peak_up_speed_mm_s > shown_up + 10) ||
        (st.peak_tilt_cdeg > shown_tilt + 200U) ||
        (st.peak_up_speed_mm_s < shown_up))      /* reset after RESET */
    {
        shown_up = st.peak_up_speed_mm_s;
        shown_tilt = st.peak_tilt_cdeg;
        tm_printf((UB*)"[S4] peaks without lift: up-speed %d mm/s (limit %u), "
                  "tilt %u deg (limit %u)\n",
                  (int)shown_up, (unsigned)s4_config.lift_speed_mm_s,
                  (unsigned)(shown_tilt / 100U),
                  (unsigned)s4_config.lift_tilt_deg);
    }
}

static void subsystem4_task(INT stacd, void *exinf)
{
    BOOL reported = FALSE;

    (void)stacd;
    (void)exinf;

    while (!b4_init(&s4_config, subsystem4_event,
                    subsystem4_urgent_brake, NULL))
    {
        if (!reported)
        {
            tm_printf((UB*)"ERROR S4: LSM303DLHC not answering at 0x%02x on "
                      "I2C1 GP26/GP27 - check wiring; retrying\n",
                      (unsigned)s4_config.i2c_addr);
            reported = TRUE;
        }
        (void)tk_dly_tsk(S4_RETRY_MS);
    }
    s4_hw_ready = TRUE;
    tm_printf((UB*)"[S4] LSM303DLHC ready on I2C1 GP26/GP27, calibrating "
              "(keep the robot still)\n");

    for (;;)
    {
        subsystem4_feed_odometry();
        b4_poll();
        subsystem4_report_peaks();
        (void)tk_dly_tsk(S4_SAMPLE_MS);
    }
}

EXPORT ER subsystem4_init(void)
{
    /* CR1: s4_config is a const initialiser now */
    return E_OK;
}

EXPORT ER subsystem4_start(void)
{
    /* Core 0: core 1 is kept busy by the S3 barcode poller (priority 16),
       which would starve this priority-18 task there. */
    s4_task_id = app_start_task(subsystem4_task, S4_PRI_MAIN, TP_PRC1,
                                "imu_task");
    return (s4_task_id < E_OK) ? (ER)s4_task_id : E_OK;
}
