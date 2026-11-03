/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make ... TEST=1
 *	Record results in testing/subsystem1/TEST_LOG.md.  IDs: S1-U01, ...
 *
 *	These run in the initial task while the system runs, so they only
 *	use the pure codec (tlm_codec.c) and read-only views. They never call
 *	tlm_bridge_*() functions: those belong to the WiFi service task.
 */

#include "subsystem1.h"
#include "s1_internal.h"
#include "tlm_codec.h"
#include "../test_report.h"

#if APP_TEST

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

	/* --- frame sizes (Minimum-Bit) ----------------------------------- */
	s.speed_r_mm_s = 5000u;			/* too big: must clamp, not overflow */
	TEST_CHECK("S1-U09", tlm_encode_status(&s, buf, sizeof(buf)) == TLM_STATUS_LEN,
		   "STATUS frame is 17 bytes");
	TEST_CHECK("S1-U10", tlm_encode_status(&s, buf, 10u) == 0u,
		   "encoder refuses a too-small buffer");

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
