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
 *	         b2_get_status(), b2_authorized_reset_while_stopped(),
 *	         b2_goal_start() / b2_goal_check(), and the write-up's
 *	         b2_move_forward(), b2_move_backward(), b2_turn_left(),
 *	         b2_turn_right()
 *	       All of these are safe from any task on either core.
 *
 *	Calibration (ticks per wheel turn, wheel size, track): motion_cal.h
 */

#ifndef B2_TKERNEL_ADAPTER_H
#define B2_TKERNEL_ADAPTER_H

#include "../app.h"
#include "motion.h"

#define S2_PRI_MAIN		14	/* priority band 14-15 (TEAM.md) */
#define S2_POLL_PERIOD_MS	10	/* tk_dly_tsk() argument; on the 10 ms
					   kernel tick the real loop period
					   is ~20 ms (see b2_poll in motion.h) */
#define S2_ENC_INTPRI		2	/* NVIC priority of the encoder IRQ */

/* Controller gains, Q8 (value / 256; units in b2_config_t, motion.h).
 * The defaults below are what the robot runs. For PID tuning runs, override
 * them from the build command instead of editing this file, e.g.
 *   make TEST=1 NET=lwip MQTT=1 S2_CAL=1 S2_KP=192 S2_KI=8 S2_KD=32
 * (build_make/pico_rp2040.mk). The S2_CAL run prints the gains it used. */
#ifndef B2_KP_Q8
#define B2_KP_Q8		(128U)	/* P: 0.5 permil per mm/s of error */
#endif
#ifndef B2_KI_Q8
#define B2_KI_Q8		(8U)	/* I: per ~20 ms loop */
#endif
#ifndef B2_KD_Q8
#define B2_KD_Q8		(0U)	/* D: off until tuning shows it helps */
#endif
#ifndef B2_STRAIGHT_KP_Q8
#define B2_STRAIGHT_KP_Q8	(256U)	/* straight-line correction */
#endif

/* TRUE once the motion task has configured the hardware (b2_init OK). */
IMPORT BOOL s2_ready(void);

#endif /* B2_TKERNEL_ADAPTER_H */
