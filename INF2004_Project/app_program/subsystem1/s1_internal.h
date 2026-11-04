/*
 *	s1_internal.h
 *	Subsystem 1 -- shared between OUR OWN files only (not for other
 *	subsystems, not for mqtt_utk.c).
 */

#ifndef S1_INTERNAL_H
#define S1_INTERNAL_H

#include "../app.h"
#include "tlm_config.h"

/* Outgoing frames: telemetry task -> WiFi service task.
 * Each message = 1 topic-ID byte (TLM_TID_...) + payload.
 * Created in subsystem1_init(). Read only by tlm_bridge.c. */
IMPORT ID s1_mbf_out;

/* Accepted commands: WiFi service task -> telemetry task.
 * The WiFi task must never wait, so it only queues the command (TMO_POL)
 * and wakes our task; our task then calls the other subsystems.
 * Created in subsystem1_init(). */
typedef struct {
	UB	code;			/* tlm_cmd_code_t */
	UH	seq;
	UW	t_rx_us;		/* when the WiFi task received it */
} S1_CMD;

IMPORT ID s1_mbf_cmd;
IMPORT ID s1_tid_tlm;			/* our telemetry task, for tk_wup_tsk */

/* Counters kept by the bridge (written only by the WiFi service task,
 * read by our telemetry task for the heartbeat). */
typedef struct {
	UW	cmds_ok;
	UW	cmds_rejected;
	UW	queue_full;
	UW	reconnects;
	UW	last_cmd_us;		/* command received -> handed over */
} S1_STATS;

/* Read-only views of the bridge state, safe from our telemetry task. */
IMPORT UH   tlm_bridge_session(void);	/* 0 = not connected */
IMPORT void tlm_bridge_get_stats(S1_STATS *p_out);

/* The telemetry task body (task_telemetry.c). */
IMPORT void s1_telemetry_task(INT stacd, void *exinf);

#endif /* S1_INTERNAL_H */
