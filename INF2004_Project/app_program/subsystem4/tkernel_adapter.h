/*
 *      tkernel_adapter.h
 *      Subsystem 4 -- GY-511 / LSM303DLHC accelerometer: lift/overturn stop,
 *      hump profile, turn rate (from wheel speeds), motion class.
 *      PUBLIC interface: other subsystems include "../subsystem4/tkernel_adapter.h".
 *
 *      Pins (TEAM.md section 2): GP26 = I2C1 SDA, GP27 = I2C1 SCL.
 *      Peripheral: I2C1, owned by imu_task (driver: drv_i2c1.c).
 *
 *      IF-04  S4 -> S2   on lifted / overturned / sensor fault S4 calls
 *                        b2_emergency_brake()
 *      IF-05  S2 -> S4   imu_task reads b2_get_status() odometry and wheel
 *                        speeds and feeds b4_update_travel_mm() (hump height)
 *                        and b4_update_wheel_speeds() (turn rate)
 *      Status for anyone: b4_get_status() (safe from any task, either core).
 */

#ifndef B4_TKERNEL_ADAPTER_H
#define B4_TKERNEL_ADAPTER_H

#include "../app.h"
#include "../subsystem2/motion_cal.h"
#include "imu.h"

#define S4_PRI_MAIN             18      /* priority band 18-19 (TEAM.md) */
#define S4_SAMPLE_MS            10      /* accelerometer poll period */
#define S4_RETRY_MS             1000    /* retry period while the sensor is absent */
/* Wheel track: taken from S2's calibration so the two cannot drift apart
   (was a separate 120 while S2 re-measured 110 on 20 Nov). */
#define S4_WHEEL_TRACK_MM       (B2_CAL_WHEEL_TRACK_MM)
#define S4_WHEELBASE_MM         131     /* EFFECTIVE wheelbase, CALIBRATED: hump
                       height = this x sin(peak tilt), so
                       it scales every reading. 17 Nov:
                       6 crossings of the 10 mm course hump
                       read 6-10 mm (mean 7.3) at 96, so
                       96 x 10 / 7.3 = 131. It absorbs the
                       filter lag on the tilt peak; re-check
                       if the IMU mount or speed changes. */

/* TRUE once the LSM303DLHC answered and was configured. */
IMPORT BOOL s4_ready(void);

#endif /* B4_TKERNEL_ADAPTER_H */
