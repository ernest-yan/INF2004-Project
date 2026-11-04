/*
 *	subsystem1.h
 *	Subsystem 1 -- WiFi telemetry and remote commands (Buddy 1).
 *	Student ID: 2502568.
 *
 *	Public interface: only what OTHER subsystems may use.
 *	(WiFi, lwIP and MQTT themselves belong to the kernel's cyw43_utk_task;
 *	 see TEAM.md section 3.)
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

/* Priority band 12-13 by default (TEAM.md section 4). Telemetry is not
 * time-critical: propose to the team that it gets the LOWEST band. */
#define S1_PRI_TLM		13
#define S1_TLM_STACK		4096	/* bytes: tm_sprintf + JSON buffer */

/* Is the car connected to the broker right now? Safe from any task. */
IMPORT BOOL s1_is_online(void);

/* Has the operator sent START (and no STOP / E-STOP / RESET since)?
 * For S3's line_follow_task to check before driving. Safe from any
 * task, either core. At power-on: FALSE in MQTT builds (wait for START),
 * TRUE otherwise (nobody can send START, so drive as before). */
IMPORT BOOL s1_mission_go(void);

#if TM_MQTT
#define S1_GO_AT_POWER_ON	FALSE
#else
#define S1_GO_AT_POWER_ON	TRUE
#endif

#endif /* SUBSYSTEM1_H */
