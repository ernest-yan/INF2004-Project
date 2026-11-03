/*
 *	tkernel_adapter.h
 *	Subsystem 2 -- motion control (2 DC motors + 2 single-channel encoders).
 *	PUBLIC interface: other subsystems include "../subsystem2/tkernel_adapter.h".
 *
 *	Pins (TEAM.md section 2):
 *	  GP6  left encoder   GP7  right encoder   (rising-edge IRQ, IO_IRQ_BANK0)
 *	  GP8/GP9   left motor PWM A/B   (PWM slice 4)
 *	  GP10/GP11 right motor PWM A/B  (PWM slice 5)
 *
 *	IF-03  any -> S2   function API in motion.h:
 *	         b2_submit_command(), b2_stop(), b2_emergency_brake(),
 *	         b2_get_status(), b2_authorized_reset_while_stopped()
 *	       All of these are safe from any task on either core.
 */

#ifndef B2_TKERNEL_ADAPTER_H
#define B2_TKERNEL_ADAPTER_H

#include "../app.h"
#include "motion.h"

#define S2_PRI_MAIN		14	/* priority band 14-15 (TEAM.md) */
#define S2_POLL_PERIOD_MS	10	/* motion control loop period */
#define S2_ENC_INTPRI		2	/* NVIC priority of the encoder IRQ */

/* TRUE once the motion task has configured the hardware (b2_init OK). */
IMPORT BOOL s2_ready(void);

#endif /* B2_TKERNEL_ADAPTER_H */
