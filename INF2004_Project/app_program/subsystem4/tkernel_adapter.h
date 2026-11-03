/*
 *	tkernel_adapter.h
 *	Subsystem 4 -- GY-511 / LSM303DLHC accelerometer: impact + hump profile.
 *	PUBLIC interface: other subsystems include "../subsystem4/tkernel_adapter.h".
 *
 *	Pins (TEAM.md section 2): GP26 = I2C1 SDA, GP27 = I2C1 SCL.
 *	Peripheral: I2C1, owned by imu_task (driver: drv_i2c1.c).
 *
 *	IF-04  S4 -> S2   on impact / sensor fault S4 calls b2_emergency_brake()
 *	IF-05  S2 -> S4   imu_task reads b2_get_status() odometry and feeds
 *	                  b4_update_travel_mm() (hump height estimate)
 *	Status for anyone: b4_get_status() (safe from any task, either core).
 */

#ifndef B4_TKERNEL_ADAPTER_H
#define B4_TKERNEL_ADAPTER_H

#include "../app.h"
#include "imu.h"

#define S4_PRI_MAIN		18	/* priority band 18-19 (TEAM.md) */
#define S4_SAMPLE_MS		10	/* accelerometer poll period */
#define S4_RETRY_MS		1000	/* retry period while the sensor is absent */

/* TRUE once the LSM303DLHC answered and was configured. */
IMPORT BOOL s4_ready(void);

#endif /* B4_TKERNEL_ADAPTER_H */
