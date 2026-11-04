/*
 *	subsystem1.c
 *	Subsystem 1 -- WiFi telemetry and remote commands (Buddy 1).
 *
 *	subsystem1_init():  create our kernel objects
 *	subsystem1_start(): create and start our telemetry task
 *
 *	How the pieces fit (TEAM.md sections 3 and 5):
 *
 *	  s1_telemetry_task (ours, priority S1_PRI_TLM)
 *	      reads every buddy's status, packs frames
 *	      --> s1_mbf_out (message buffer, owned by us) -->
 *	  cyw43_utk_task (kernel's WiFi task) runs mqtt_utk.c, which calls
 *	      tlm_bridge.c to empty s1_mbf_out (TMO_POL) and publishes.
 *	  Commands come back the other way: mqtt_utk.c -> tlm_bridge.c
 *	      --> s1_mbf_cmd (owned by us) + tk_wup_tsk --> s1_telemetry_task,
 *	      which acts on them (team_adapters.c). The WiFi task never waits.
 */

#include "subsystem1.h"
#include "s1_internal.h"

EXPORT ID s1_mbf_out;
EXPORT ID s1_mbf_cmd;
EXPORT ID s1_tid_tlm;

EXPORT ER subsystem1_init(void)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)TLM_MBF_BYTES;	/* total bytes it can hold */
	cmbf.maxmsz = (INT)TLM_MBF_MSG_MAX;	/* largest single message */
	s1_mbf_out = tk_cre_mbf(&cmbf);
	if (s1_mbf_out < E_OK) {
		return s1_mbf_out;
	}

	cmbf.bufsz  = (SZ)(TLM_CMD_QUEUE_LEN * S1_CMD_SLOT_BYTES);
	cmbf.maxmsz = (INT)sizeof(S1_CMD);
	s1_mbf_cmd = tk_cre_mbf(&cmbf);
	if (s1_mbf_cmd < E_OK) {
		return s1_mbf_cmd;
	}
	return E_OK;
}

EXPORT ER subsystem1_start(void)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	/* Own tk_cre_tsk() instead of app_start_task(): we need a bigger
	   stack than the shared TASK_STACK_SIZE (JSON mode + tm_sprintf). */
	ctsk.task    = (FP)s1_telemetry_task;
	ctsk.itskpri = S1_PRI_TLM;
	ctsk.stksz   = S1_TLM_STACK;
	ctsk.tskatr  = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
	/* Core 0 only: core 1 belongs to S3's barcode task, which times bars
	   in microseconds and is mis-measured if we preempt it. */
	ctsk.tskatr |= TA_ASSPRC;
	ctsk.assprc  = TP_PRC1;
#endif

	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, "telemetry");
		return tid;
	}
	s1_tid_tlm = tid;
	er = tk_sta_tsk(tid, 0);
	CHECK(er, "telemetry");
	return er;
}
