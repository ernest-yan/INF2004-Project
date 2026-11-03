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
 */
#include "tkernel_adapter.h"
#include "../subsystem2/tkernel_adapter.h"

static ID s4_task_id;
static b4_config_t s4_config;
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
}

static void subsystem4_task(INT stacd, void *exinf)
{
    BOOL reported = FALSE;

    (void)stacd;
    (void)exinf;

    while (!b4_init(&s4_config, NULL, subsystem4_urgent_brake, NULL))
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
        (void)tk_dly_tsk(S4_SAMPLE_MS);
    }
}

EXPORT ER subsystem4_init(void)
{
    s4_config.i2c_addr = B4_ACCEL_ADDR_DEFAULT;
    s4_config.i2c_speed_khz = 100U;
    s4_config.impact_threshold_mg = 600U; /* provisional; tune on course */
    s4_config.forward_axis_sign = 1U;     /* verify sensor orientation */
    s4_config.calibration_samples = 50U;
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
