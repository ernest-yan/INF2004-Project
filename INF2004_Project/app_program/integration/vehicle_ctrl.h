/*
 *	vehicle_ctrl.h
 *	Team-level integration task ("VehicleController", minimal safe version).
 *
 *	Shared file: changed only by team agreement (like app_main.c / app.h).
 *
 *	What it does today (the glue the subsystems already expect):
 *	  - consumes S5's event buffer mbf_s5_events (IF-02) so it never fills up
 *	  - S5 STOP_REQUIRED / FAULT        -> b2_stop()
 *	  - wheels confirmed still (S2 odometry quiet) while S5 waits
 *	                                     -> subsystem5_confirm_robot_stopped()
 *	    so S5 can run its servo scan instead of timing out after 1.5 s
 *	  - checks at boot that the RP2040 microsecond TIMER really runs at 1 MHz
 *	  - prints a one-line health summary every VC_HEALTH_PRINT_MS
 *
 *	The mission logic lives in the subsystems (APP_MISSION_ENABLE in app.h):
 *	  - S3 line_barcode.c: line following and barcode turns
 *	  - S5 ultrasonic.c:   obstacle bypass after PROFILE_READY
 */

#ifndef VEHICLE_CTRL_H
#define VEHICLE_CTRL_H

#include "../app.h"

#define VC_PRI			11	/* above every subsystem band (12-21) */
#define VC_PERIOD_MS		20	/* max wait for an S5 event per loop */
#define VC_STILL_MS		100	/* no encoder edges for this long = still */
#if APP_TEST
#define VC_HEALTH_PRINT_MS	0	/* keep test logs clean */
#else
#define VC_HEALTH_PRINT_MS	5000	/* 0 = never */
#endif

IMPORT ER vc_init(void);
IMPORT ER vc_start(void);

#endif /* VEHICLE_CTRL_H */
