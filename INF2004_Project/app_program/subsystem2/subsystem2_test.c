/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2 (motion).  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.  IDs: S2-U01, ...
 *
 *	SAFE BY DESIGN: the S2-Uxx tests never command the wheels to turn.
 *	Keep the robot's wheels off the floor anyway when running a test build.
 *
 *	CALIBRATION RUN (make TEST=1 S2_CAL=1): after the S2-Uxx tests, the
 *	S2-Cxx steps below DRIVE THE ROBOT on the floor. Procedure and how to
 *	turn the printed numbers into motion_cal.h values:
 *	testing/subsystem2/CALIBRATION.md.
 */

#include "tkernel_adapter.h"
#include "b2_math.h"
#include "motion_cal.h"
#include "../test_report.h"

#ifndef S2_CAL_RUN
#define S2_CAL_RUN	0	/* 1: make TEST=1 S2_CAL=1 (drives the robot) */
#endif
#if S2_CAL_RUN
#include "../subsystem4/tkernel_adapter.h"	/* reset S4's lift latch */
#endif

#if APP_TEST

#if S2_CAL_RUN

#define S2_CAL_LOOP_MS		20	/* matches the motion loop */
#define S2_CAL_HAND_MS		30000	/* time for 10 hand turns of a wheel */
#define S2_CAL_PLACE_MS		15000	/* time to put the robot down */
#define S2_CAL_MEASURE_MS	20000	/* time to measure and re-place */
#define S2_CAL_MOVE_MM		1000U
#define S2_CAL_TURN_DEG		360U
#define S2_CAL_STEP_MM_S	150	/* step-response target */
#define S2_CAL_STEP_MS		2000

/* Wait `ms`, printing a countdown every 5 s. */
static void s2_cal_wait(const char *p_what, INT ms)
{
	INT left;

	for (left = ms; left > 0; left -= 5000) {
		tm_printf((UB*)"[S2-CAL] %s: %d s\n", p_what, left / 1000);
		tk_dly_tsk((left >= 5000) ? 5000 : left);
	}
}

/* Count the edges of one wheel while a person turns it 10 times. */
static void s2_cal_hand_turns(const char *p_side, BOOL b_left)
{
	b2_status_t st;
	uint32_t start;
	uint32_t edges;

	b2_get_status(&st);
	start = b_left ? st.left_encoder_edges : st.right_encoder_edges;
	tm_printf((UB*)"[S2-CAL] S2-C01 turn the %s wheel EXACTLY 10 turns "
		  "by hand (use the tyre mark)\n", p_side);
	s2_cal_wait("turning time left", S2_CAL_HAND_MS);
	b2_get_status(&st);
	edges = (b_left ? st.left_encoder_edges : st.right_encoder_edges) -
		start;
	tm_printf((UB*)"[S2-CAL] S2-C01 %s: %u edges in 10 turns -> "
		  "B2_CAL_TICKS_PER_REV_%s = %u (now %u)\n", p_side,
		  (unsigned)edges, b_left ? "LEFT" : "RIGHT",
		  (unsigned)((edges + 5U) / 10U),
		  (unsigned)(b_left ? B2_CAL_TICKS_PER_REV_LEFT :
				      B2_CAL_TICKS_PER_REV_RIGHT));
}

/* Clear S4's lift latch and S2's brake after the robot was picked up. */
static BOOL s2_cal_unlatch(void)
{
	b2_status_t st;

	tk_dly_tsk(500);			/* wheels still for 200 ms+ */
	(void)b4_authorized_reset_while_stopped();
	b2_get_status(&st);
	if ((st.brake_latched != 0U) || (st.state == B2_FAULT)) {
		if (!b2_authorized_reset_while_stopped()) {
			tm_printf((UB*)"[S2-CAL] cannot reset the brake: are the "
				  "wheels still? calibration stopped\n");
			return FALSE;
		}
	}
	return TRUE;
}

/* Run one goal to the end and print what the encoders saw. */
static BOOL s2_cal_goal(const char *p_id, BOOL b_turn, BOOL b_left)
{
	b2_goal_t goal;
	b2_status_t st;
	b2_goal_result_t result = B2_GOAL_RUNNING;
	uint32_t start_l;
	uint32_t start_r;
	uint32_t edges_l;
	uint32_t edges_r;
	BOOL b_ok;

	if (!s2_cal_unlatch()) {
		return FALSE;
	}
	b2_get_status(&st);
	start_l = st.left_encoder_edges;
	start_r = st.right_encoder_edges;
	if (b_turn) {
		b_ok = b_left ? b2_turn_left(S2_CAL_TURN_DEG, &goal) :
				b2_turn_right(S2_CAL_TURN_DEG, &goal);
	} else {
		b_ok = b2_move_forward(S2_CAL_MOVE_MM, &goal);
	}
	if (!b_ok) {
		tm_printf((UB*)"[S2-CAL] %s refused by S2 (state %u)\n", p_id,
			  (unsigned)st.state);
		return FALSE;
	}
	while (result == B2_GOAL_RUNNING) {
		tk_dly_tsk(S2_CAL_LOOP_MS);
		result = b2_goal_check(&goal);
	}
	tk_dly_tsk(300);			/* let the robot settle */
	b2_get_status(&st);
	edges_l = st.left_encoder_edges - start_l;
	edges_r = st.right_encoder_edges - start_r;
	tm_printf((UB*)"[S2-CAL] %s %s: edges L %u R %u (L-R %d), encoder "
		  "distance L %d mm R %d mm\n", p_id,
		  (result == B2_GOAL_DONE) ? "DONE" : "FAILED",
		  (unsigned)edges_l, (unsigned)edges_r,
		  (int)((INT)edges_l - (INT)edges_r),
		  (int)b2_ticks_to_mm(edges_l, B2_CAL_TICKS_PER_REV_LEFT,
				      B2_CAL_WHEEL_CIRCUMFERENCE_MM),
		  (int)b2_ticks_to_mm(edges_r, B2_CAL_TICKS_PER_REV_RIGHT,
				      B2_CAL_WHEEL_CIRCUMFERENCE_MM));
	return (result == B2_GOAL_DONE) ? TRUE : FALSE;
}

/* Speed step 0 -> S2_CAL_STEP_MM_S: one CSV line per loop, for the PID
 * tuning report (grep '^S2-STEP' in the console log). */
static void s2_cal_step_response(void)
{
	b2_command_t cmd = {0};
	b2_status_t st;
	INT t_ms;

	if (!s2_cal_unlatch()) {
		return;
	}
	/* Which gains this step response belongs to (Q8 values). */
	tm_printf((UB*)"S2-GAINS,kp_q8=%u,ki_q8=%u,kd_q8=%u,straight_kp_q8=%u\n",
		  (unsigned)B2_KP_Q8, (unsigned)B2_KI_Q8,
		  (unsigned)B2_KD_Q8, (unsigned)B2_STRAIGHT_KP_Q8);
	tm_printf((UB*)"S2-STEP,t_ms,left_mm_s,right_mm_s,left_pwm,"
		  "right_pwm,left_edges,right_edges\n");
	cmd.type = B2_CMD_SPEED;
	cmd.left_mm_s = S2_CAL_STEP_MM_S;
	cmd.right_mm_s = S2_CAL_STEP_MM_S;
	(void)b2_submit_command(&cmd);
	for (t_ms = 0; t_ms <= S2_CAL_STEP_MS; t_ms += S2_CAL_LOOP_MS) {
		b2_get_status(&st);
		tm_printf((UB*)"S2-STEP,%d,%d,%d,%u,%u,%u,%u\n", t_ms,
			  (int)b2_speed_value(st.left_speed_ob),
			  (int)b2_speed_value(st.right_speed_ob),
			  (unsigned)st.left_pwm_permil,
			  (unsigned)st.right_pwm_permil,
			  (unsigned)st.left_encoder_edges,
			  (unsigned)st.right_encoder_edges);
		tk_dly_tsk(S2_CAL_LOOP_MS);
	}
	b2_stop();
}

static void s2_calibration_run(void)
{
	tm_printf((UB*)"[S2-CAL] ===== calibration run: the robot WILL drive "
		  "(testing/subsystem2/CALIBRATION.md) =====\n");
	tm_printf((UB*)"[S2-CAL] now: ticks/rev L %u R %u, circumference %u "
		  "mm, track %u mm\n",
		  (unsigned)B2_CAL_TICKS_PER_REV_LEFT,
		  (unsigned)B2_CAL_TICKS_PER_REV_RIGHT,
		  (unsigned)B2_CAL_WHEEL_CIRCUMFERENCE_MM,
		  (unsigned)B2_CAL_WHEEL_TRACK_MM);

	/* S2-C01: encoder edges per wheel turn, wheels lifted */
	tm_printf((UB*)"[S2-CAL] lift the robot (wheels free)\n");
	s2_cal_wait("lift now", 10000);
	s2_cal_hand_turns("LEFT", TRUE);
	s2_cal_hand_turns("RIGHT", FALSE);

	/* S2-C02: straight 1 m -> circumference, and straightness */
	tm_printf((UB*)"[S2-CAL] S2-C02 put the robot on the floor at a "
		  "start mark, 1.5 m clear ahead, NOT on a black line\n");
	s2_cal_wait("drives 1000 mm in", S2_CAL_PLACE_MS);
	if (!s2_cal_goal("S2-C02 forward 1000 mm", FALSE, FALSE)) {
		return;
	}
	tm_printf((UB*)"[S2-CAL] S2-C02 measure the distance D (mm) actually "
		  "driven: B2_CAL_WHEEL_CIRCUMFERENCE_MM = %u * D / 1000. "
		  "Also note the sideways drift at 1 m.\n",
		  (unsigned)B2_CAL_WHEEL_CIRCUMFERENCE_MM);

	/* S2-C03 / S2-C04: 360 deg on the spot -> track */
	tm_printf((UB*)"[S2-CAL] S2-C03 mark the robot's heading "
		  "(tape along its side)\n");
	s2_cal_wait("turns LEFT 360 deg in", S2_CAL_MEASURE_MS);
	if (!s2_cal_goal("S2-C03 turn left 360 deg", TRUE, TRUE)) {
		return;
	}
	tm_printf((UB*)"[S2-CAL] S2-C03 measure the angle A (deg) actually "
		  "turned: B2_CAL_WHEEL_TRACK_MM = %u * 360 / A\n",
		  (unsigned)B2_CAL_WHEEL_TRACK_MM);
	s2_cal_wait("turns RIGHT 360 deg in", S2_CAL_MEASURE_MS);
	if (!s2_cal_goal("S2-C04 turn right 360 deg", TRUE, FALSE)) {
		return;
	}
	tm_printf((UB*)"[S2-CAL] S2-C04 same formula; use the mean of the "
		  "left and right results\n");

	/* S2-C05: speed step response, for the PID tuning report */
	tm_printf((UB*)"[S2-CAL] S2-C05 speed step 0 -> %d mm/s for %d ms, "
		  "2 m clear ahead\n", S2_CAL_STEP_MM_S, S2_CAL_STEP_MS);
	s2_cal_wait("drives in", S2_CAL_MEASURE_MS);
	s2_cal_step_response();
	tm_printf((UB*)"[S2-CAL] ===== calibration run finished =====\n");
}
#endif /* S2_CAL_RUN */

EXPORT void subsystem2_test(void)
{
	b2_status_t st;
	b2_command_t cmd = {0};

	TEST_INIT();

	/* S2-U01: the motion task configured the hardware */
	tk_dly_tsk(100);
	TEST_CHECK("S2-U01", s2_ready(), "b2_init done in motion_task");

	/* S2-U02: idle state after start-up */
	b2_get_status(&st);
	TEST_CHECK("S2-U02", st.state == B2_READY && st.fault_mask == 0U &&
		   st.left_pwm_permil == 0 && st.right_pwm_permil == 0,
		   "READY, no fault, both PWM outputs zero");

	/* S2-U03: distance/turn maths (20 ticks/rev, 210 mm circumference) */
	TEST_CHECK("S2-U03", b2_mm_to_ticks(210U, 20U, 210U) == 20U &&
		   b2_mm_to_ticks(105U, 20U, 210U) == 10U &&
		   b2_ticks_to_mm(20U, 20U, 210U) == 210 &&
		   b2_mm_to_ticks(10U, 0U, 210U) == 0U &&
		   b2_turn_to_mm(90U, 120U) == 94U,
		   "mm <-> encoder tick conversion, turn arc");

	/* S2-U04: out-of-range / malformed commands are refused */
	cmd.type = B2_CMD_SPEED;
	cmd.left_mm_s = 10000;
	cmd.right_mm_s = 0;
	{
		BOOL r1 = b2_submit_command(&cmd);
		cmd.type = B2_CMD_MOVE_FORWARD;
		cmd.distance_mm = 100U;
		cmd.goal_speed_mm_s = 50U;
		cmd.timeout_ms = 0U;			/* missing timeout */
		BOOL r2 = b2_submit_command(&cmd);
		TEST_CHECK("S2-U04", !r1 && !r2 && !b2_submit_command(NULL),
			   "invalid speed / zero timeout / NULL rejected");
	}

	/* S2-U07: the motion API refuses amounts it cannot do. Every call
	   here is refused, so the wheels do not move. */
	TEST_CHECK("S2-U07", !b2_move_forward(0U, NULL) &&
		   !b2_move_backward(1200U, NULL) &&
		   !b2_turn_left(0U, NULL) && !b2_turn_right(361U, NULL),
		   "move/turn API refuses 0, too far, > 360 deg");

	/* S2-U08: S5's tick scales come from S2's calibration header */
	TEST_CHECK("S2-U08", B2_CAL_TICKS_PER_M ==
		   ((B2_CAL_TICKS_PER_REV_MEAN * 1000U +
		     B2_CAL_WHEEL_CIRCUMFERENCE_MM / 2U) /
		    B2_CAL_WHEEL_CIRCUMFERENCE_MM) &&
		   B2_CAL_TURN_TICKS_PER_DEG_X10 > 0U,
		   "one calibration source for S2 and S5");

	/* S2-U05: emergency brake latches and blocks new commands */
	b2_emergency_brake();
	b2_get_status(&st);
	cmd.type = B2_CMD_SPEED;
	cmd.left_mm_s = 0;
	cmd.right_mm_s = 0;
	TEST_CHECK("S2-U05", st.brake_latched == 1U && st.state == B2_BRAKED &&
		   !b2_submit_command(&cmd),
		   "brake latched, commands refused");

	/* S2-U06: authorised reset clears the latch when the wheels are still */
	tk_dly_tsk(300);
	{
		BOOL ok = b2_authorized_reset_while_stopped();
		b2_get_status(&st);
		TEST_CHECK("S2-U06", ok && st.brake_latched == 0U &&
			   st.state == B2_READY,
			   "authorised reset returns to READY");
	}

	TEST_SUMMARY("subsystem2");

#if S2_CAL_RUN
	s2_calibration_run();
#endif
}

#endif /* APP_TEST */
