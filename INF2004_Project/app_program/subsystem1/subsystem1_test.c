/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make ... TEST=1
 *	Record results in testing/subsystem1/TEST_LOG.md.  IDs: S1-U01, ...
 *
 *	These run in the initial task while the system runs, so they only
 *	use the pure codec (tlm_codec.c) and read-only views. They never call
 *	tlm_bridge_*() functions: those belong to the WiFi service task.
 *	They never send a command or touch another subsystem's outputs:
 *	S1 runs first, and S2's tests expect an untouched, unbraked robot.
 */

#include "subsystem1.h"
#include "s1_internal.h"
#include "team_interfaces.h"
#include "tlm_codec.h"
#include "../subsystem2/tkernel_adapter.h"
#include "../test_report.h"

#if APP_TEST

#define S2_READY_WAIT_MS	3000	/* S2 sets up its hardware in its own task */
#define LOAD_WINDOW_MS		3000	/* watch S2 while telemetry runs */
#define LOOP_BUDGET_US		2000	/* S2 polls every 10 ms, late after 15 ms */
#define CORE0_MASK		(1u << 1)	/* tk_get_prc(): 1 = core 0 */
#define KNOWN_FAULTS		(TLM_FAULT_S2 | TLM_FAULT_S3 | TLM_FAULT_S4 | TLM_FAULT_S5)

LOCAL BOOL wait_s2_ready(void)
{
	INT	waited;

	for (waited = 0; !s2_ready() && (waited < S2_READY_WAIT_MS); waited += 100) {
		tk_dly_tsk(100);
	}
	return s2_ready();
}

LOCAL UW make_cmd(UB code, UH session, UH seq, UB *p_buf)
{
	tlm_cmd_t c;

	c.code    = code;
	c.session = session;
	c.seq     = seq;
	return tlm_encode_cmd(&c, p_buf, TLM_CMD_LEN);
}

EXPORT void subsystem1_test(void)
{
	UB		buf[32];
	tlm_cmd_t	out;
	UH		last = 0u;
	const UH	sess = 0x4A3Bu;
	tlm_status_t	s = {0};
	T_RMBF		rmbf;

	TEST_INIT();

	/* --- setup ------------------------------------------------------ */
	TEST_CHECK("S1-U01", tk_ref_mbf(s1_mbf_out, &rmbf) == E_OK,
		   "outgoing message buffer exists");

	/* --- command wire format (MQTT_PROTOCOL.md) --------------------- */
	(void)make_cmd(TLM_CMD_START, sess, 1u, buf);
	TEST_CHECK("S1-U02", buf[0] == 0x11u && buf[1] == 0x4Au && buf[2] == 0x3Bu
		   && buf[3] == 0x00u && buf[4] == 0x01u,
		   "START packs to 11 4A 3B 00 01");

	/* --- command checks --------------------------------------------- */
	TEST_CHECK("S1-U03", tlm_cmd_validate(buf, TLM_CMD_LEN, sess, &last, &out)
		   == TLM_RES_ACCEPTED && last == 1u, "valid command accepted");
	TEST_CHECK("S1-U04", tlm_cmd_validate(buf, TLM_CMD_LEN, sess, &last, &out)
		   == TLM_RES_STALE_SEQ, "replayed command rejected (STALE_SEQ)");

	(void)make_cmd(TLM_CMD_START, 0x1111u, 2u, buf);
	TEST_CHECK("S1-U05", tlm_cmd_validate(buf, TLM_CMD_LEN, sess, &last, &out)
		   == TLM_RES_BAD_SESSION && last == 1u,
		   "old-session / retained command rejected, seq unchanged");

	(void)make_cmd(TLM_CMD_STOP, 0u, 9u, buf);
	TEST_CHECK("S1-U06", tlm_cmd_validate(buf, TLM_CMD_LEN, 0u, &last, &out)
		   == TLM_RES_BAD_SESSION, "offline (session 0) rejects everything");

	TEST_CHECK("S1-U07", tlm_cmd_validate(buf, 4u, sess, &last, &out)
		   == TLM_RES_BAD_LEN, "wrong length rejected");

	(void)make_cmd(9u, sess, 3u, buf);
	TEST_CHECK("S1-U08", tlm_cmd_validate(buf, TLM_CMD_LEN, sess, &last, &out)
		   == TLM_RES_BAD_CMD, "unknown command code rejected");

	/* RESET / MAP: format checks only. Nothing is braked or reset, so
	   the other subsystems' tests are not disturbed. */
	(void)make_cmd(TLM_CMD_RESET, sess, 3u, buf);
	TEST_CHECK("S1-U19", tlm_cmd_validate(buf, TLM_CMD_LEN, sess, &last, &out)
		   == TLM_RES_ACCEPTED && out.code == TLM_CMD_RESET,
		   "RESET command accepted");
	{
		tlm_cmd_t	m;

		m.code    = TLM_CMD_MAP;
		m.session = sess;
		m.seq     = 4u;
		m.map[0]  = 'F'; m.map[1] = 'U'; m.map[2] = 'L'; m.map[3] = 'R';
		(void)tlm_encode_cmd(&m, buf, TLM_CMD_MAP_LEN);
		TEST_CHECK("S1-U20", tlm_cmd_validate(buf, TLM_CMD_MAP_LEN, sess, &last, &out)
			   == TLM_RES_ACCEPTED && out.map[0] == 'F' && out.map[3] == 'R',
			   "MAP command accepted, letters kept in order");

		m.seq    = 5u;
		m.map[1] = 'F';			/* same letter twice */
		(void)tlm_encode_cmd(&m, buf, TLM_CMD_MAP_LEN);
		TEST_CHECK("S1-U21", tlm_cmd_validate(buf, TLM_CMD_MAP_LEN, sess, &last, &out)
			   == TLM_RES_BAD_CMD, "MAP with a repeated letter rejected");
	}
	TEST_CHECK("S1-U22", s1_mission_go() == FALSE,
		   "no START at power-on: mission go flag is off");

	/* --- frame sizes (Minimum-Bit) ----------------------------------- */
	s.speed_r_mm_s = 5000u;			/* too big: must clamp, not overflow */
	TEST_CHECK("S1-U09", tlm_encode_status(&s, buf, sizeof(buf)) == TLM_STATUS_LEN,
		   "STATUS frame is 17 bytes");
	TEST_CHECK("S1-U10", tlm_encode_status(&s, buf, 10u) == 0u,
		   "encoder refuses a too-small buffer");

	/* --- command path: WiFi task -> our task (fix 2) ---------------- */
	TEST_CHECK("S1-U11", tk_ref_mbf(s1_mbf_cmd, &rmbf) == E_OK
		   && rmbf.maxmsz == (INT)sizeof(S1_CMD)
		   && rmbf.frbufsz == (SZ)(TLM_CMD_QUEUE_LEN * S1_CMD_SLOT_BYTES),
		   "command queue exists and holds exactly TLM_CMD_QUEUE_LEN commands");

	/* --- our task: priority and core (fix 1) ------------------------ */
	{
		T_RTSK	rtsk;
		S1_DIAG	d;

		tk_dly_tsk(2 * TLM_TASK_TICK_MS);	/* let it loop at least once */
		s1_tlm_get_diag(&d);
		TEST_CHECK("S1-U12", tk_ref_tsk(s1_tid_tlm, &rtsk) == E_OK
			   && rtsk.tskbpri == S1_PRI_TLM && d.loops > 0u,
			   "telemetry task running at S1_PRI_TLM");
#if TK_SUPPORT_SMP
		TEST_CHECK("S1-U13", d.cores_seen == CORE0_MASK,
			   "telemetry task only ever ran on core 0 (S3 owns core 1)");
#endif
	}

	/* --- real teammate data (fix 3); read-only, robot at rest ------- */
	{
		vehicle_status_t	v;
		motion_status_t		m;

		TEST_CHECK("S1-U14", wait_s2_ready(), "S2 ready within 3 s");
		vehicle_get_status(&v);
		motion_get_status(&m);
		TEST_CHECK("S1-U15", v.state != (uint8_t)VEH_STATE_INIT
			   && v.state <= (uint8_t)VEH_STATE_SAFE_STOP
			   && (v.fault_mask & (uint8_t)~KNOWN_FAULTS) == 0u,
			   "vehicle state worked out from S2-S5, faults only in bits 2-5");
		TEST_CHECK("S1-U16", m.speed_l_mm_s == 0u && m.speed_r_mm_s == 0u,
			   "real S2 speeds read 0 with the robot at rest");
	}

	/* --- we do not slow the motors (fix 6) --------------------------- */
	{
		b2_status_t	before, after;
		S1_DIAG		d;

		tm_printf((UB*)"[TEST] S1 note: telemetry %s for S1-U17/U18\n",
			  s1_is_online()? "ONLINE": "OFFLINE (start the broker for a real load test)");
		b2_get_status(&before);
		tk_dly_tsk(LOAD_WINDOW_MS);
		b2_get_status(&after);
		s1_tlm_get_diag(&d);
		tm_printf((UB*)"[TEST] S1 note: S2 deadline misses +%d, S1 loop max %d us\n",
			  (int)(after.deadline_misses - before.deadline_misses),
			  (int)d.loop_max_us);
		TEST_CHECK("S1-U17", after.deadline_misses == before.deadline_misses,
			   "no S2 deadline misses in 3 s with telemetry running");
		TEST_CHECK("S1-U18", d.loop_max_us < LOOP_BUDGET_US,
			   "telemetry loop always under 2 ms");
	}

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
