/*
 *	vehicle_ctrl.c
 *	Team-level integration task -- see vehicle_ctrl.h for what it does and
 *	what is still left to the team.
 *
 *	Runs on core 0 (TP_PRC1).  Uses only the public interfaces of the
 *	subsystems; it touches no pin or peripheral itself.
 */

#include "vehicle_ctrl.h"
#include "../app_hw.h"
#include "../subsystem2/tkernel_adapter.h"
#include "../subsystem3/line_barcode.h"
#include "../subsystem4/tkernel_adapter.h"
#include "../subsystem5/ultrasonic.h"

static ID	vc_tid;

/* ------------------------------------------------------------------ */

static void vc_check_timer(void)
{
	const uint32_t t0 = app_time_us_32();
	uint32_t dt;

	tk_dly_tsk(100);
	dt = app_time_us_32() - t0;
	if ((dt < 90000u) || (dt > 110000u)) {
		tm_printf((UB*)"WARNING VC: RP2040 TIMER gave %u us for a 100 ms "
			  "delay (expected ~100000). S3 barcode widths, S5 echo "
			  "distances and S2 speeds will be wrong.\n", (unsigned)dt);
	} else {
		tm_printf((UB*)"[VC] microsecond timer OK (%u us / 100 ms)\n",
			  (unsigned)dt);
	}
}

static BOOL vc_motion_active(const b2_status_t *st)
{
	return (st->state == B2_TRACK_SPEED) || (st->state == B2_EXECUTE_GOAL);
}

static void vc_handle_s5_event(const subsystem5_event_t *ev)
{
	switch (ev->type) {
	case subsystem5_EVENT_STOP_REQUIRED:
		b2_stop();
		tm_printf((UB*)"[VC] S5 obstacle at %u mm -> stop\n",
			  (unsigned)ev->front_mm);
		break;
	case subsystem5_EVENT_PROFILE_READY:
		tm_printf((UB*)"[VC] S5 profile: closest %u mm @ %u deg, width ~%u mm, "
			  "side %s. Bypass not implemented: holding stopped.\n",
			  (unsigned)ev->profile.closest_mm,
			  (unsigned)ev->profile.closest_angle_deg,
			  (unsigned)ev->profile.approximate_width_mm,
			  (ev->profile.candidate_side == subsystem5_SIDE_LEFT) ? "LEFT" :
			  (ev->profile.candidate_side == subsystem5_SIDE_RIGHT) ? "RIGHT" :
			  "NONE");
		break;
	case subsystem5_EVENT_FAULT:
	default:
		b2_stop();
		tm_printf((UB*)"[VC] S5 fault %d -> stop\n", (int)ev->fault);
		break;
	}
}

static void vc_print_health(void)
{
	b2_status_t s2;
	b4_status_t s4;
	subsystem5_status_t s5;
	LineBarcodeData_t s3;

	b2_get_status(&s2);
	b4_get_status(&s4);
	subsystem5_get_status(&s5);
	s3 = get_line_barcode_data_safe();

	tm_printf((UB*)"[VC] S2 %s st=%d flt=%d enc=%u/%u | S3 L=%u C=%u R=%u line=%u"
		  " | S4 %s st=%d pitch=%dcdeg | S5 st=%d front=%u%s\n",
		  s2_ready() ? "ok" : "--", (int)s2.state, (int)s2.fault,
		  (unsigned)s2.left_encoder_edges, (unsigned)s2.right_encoder_edges,
		  (unsigned)s3.ir_left_raw, (unsigned)s3.ir_center_raw,
		  (unsigned)s3.ir_right_raw, (unsigned)s3.line_detected,
		  s4_ready() ? "ok" : "--", (int)s4.state,
		  (int)s4.pitch_cdeg,
		  (int)s5.state, (unsigned)s5.front_mm,
		  s5.front_valid ? "mm" : "(invalid)");
}

static void vc_task(INT stacd, void *exinf)
{
	subsystem5_event_t	ev;
	subsystem5_status_t	s5;
	b2_status_t		s2;
	uint32_t		last_edges = 0u;
	uint32_t		still_since = 0u;
	uint32_t		last_health = 0u;
	char			ch;

	(void)stacd; (void)exinf;

	vc_check_timer();

	while (1) {
		/* 1. S5 events (IF-02).  This is also the loop's pacing wait. */
		if (tk_rcv_mbf(mbf_s5_events, &ev, VC_PERIOD_MS) ==
		    (INT)sizeof(ev)) {
			vc_handle_s5_event(&ev);
		}

		/* 2. decoded barcode characters (IF-01) */
		while (tk_rcv_mbf(mbf_barcode, &ch, TMO_POL) == 1) {
			tm_printf((UB*)"[VC] barcode '%c' -> action %d "
				  "(not executed yet)\n", ch,
				  (int)get_action_for_character(ch));
		}

		/* 3. S5 snapshot is authoritative (an event can be lost):
		      never let the wheels run while S5 says stop. */
		subsystem5_get_status(&s5);
		b2_get_status(&s2);
		if ((s5.stop_required != 0U) && vc_motion_active(&s2)) {
			b2_stop();
		}

		/* 4. tell S5 the robot is still, so it may start its scan */
		{
			const uint32_t now = app_time_us_32();
			const uint32_t edges = s2.left_encoder_edges +
					       s2.right_encoder_edges;
			if ((edges != last_edges) || vc_motion_active(&s2)) {
				last_edges = edges;
				still_since = now;
			}
			if ((s5.state == subsystem5_STATE_WAIT_STOP) &&
			    (!s2_ready() ||
			     ((now - still_since) >= (VC_STILL_MS * 1000u)))) {
				subsystem5_confirm_robot_stopped();
			}

#if VC_HEALTH_PRINT_MS > 0
			if ((now - last_health) >= (VC_HEALTH_PRINT_MS * 1000u)) {
				last_health = now;
				vc_print_health();
			}
#else
			(void)last_health;
			(void)vc_print_health;
#endif
		}
	}
}

/* ------------------------------------------------------------------ */

EXPORT ER vc_init(void)
{
	return E_OK;	/* uses the subsystems' objects; creates none */
}

EXPORT ER vc_start(void)
{
	vc_tid = app_start_task(vc_task, VC_PRI, TP_PRC1, "vehicle_ctrl");
	return (vc_tid < E_OK) ? (ER)vc_tid : E_OK;
}
