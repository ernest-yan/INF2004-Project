/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2 (motion).  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.  IDs: S2-U01, ...
 *
 *	SAFE BY DESIGN: no test here commands the wheels to turn.  Keep the
 *	robot's wheels off the floor anyway when running a test build.
 */

#include "tkernel_adapter.h"
#include "math.h"
#include "../test_report.h"

#if APP_TEST

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
	TEST_CHECK("S2-U02", st.state == B2_READY && st.fault == B2_FAULT_NONE &&
		   st.left_pwm_permil == 0 && st.right_pwm_permil == 0,
		   "READY, no fault, both PWM outputs zero");

	/* S2-U03: distance/turn maths (20 ticks/rev, 210 mm circumference) */
	TEST_CHECK("S2-U03", b2_mm_to_ticks(210U, 20U, 210U) == 20U &&
		   b2_mm_to_ticks(105U, 20U, 210U) == 10U &&
		   b2_ticks_to_mm(20U, 20U, 210U) == 210 &&
		   b2_mm_to_ticks(10U, 0U, 210U) == 0U,
		   "mm <-> encoder tick conversion");

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
}

#endif /* APP_TEST */
