/*
 *	tlm_bridge.c
 *	Subsystem 1 -- the network side of telemetry.
 *
 *	RUNS INSIDE THE KERNEL'S WiFi SERVICE TASK (cyw43_utk_task), because
 *	the only caller is mqtt_utk.c. So:
 *	  - never block here: every kernel call uses TMO_POL;
 *	  - never call lwIP/MQTT here: return bytes, mqtt_utk.c publishes;
 *	  - keep it short: this task also services the radio every 2 ms.
 *
 *	Command rules (MQTT_PROTOCOL.md): every command carries the current
 *	session number and a rising sequence number. A new session is picked
 *	for every connection, so anything sent before a dropout, or kept by
 *	the broker as "retained", is rejected.
 */

#include "../app.h"
#include "s1_internal.h"
#include "team_interfaces.h"
#include "tlm_bridge.h"
#include "tlm_codec.h"

IMPORT uint64_t time_us_64(void);	/* pico-sdk timer, always running */

/* State: written only by the WiFi service task (single writer). */
LOCAL volatile UH	g_session;	/* 0 = offline: reject everything */
LOCAL UH		g_next_session;	/* chosen while connecting */
LOCAL UH		g_last_seq;
LOCAL UB		g_ack_seq;
LOCAL BOOL		g_ever_online;
LOCAL volatile S1_STATS	g_stats;
LOCAL UB		g_rx[TLM_MBF_MSG_MAX];	/* scratch for one frame */

/* A new random-looking, non-zero session number, different from the last. */
LOCAL UH new_session(UH old)
{
	uint64_t t = time_us_64();
	UH	 s = (UH)((t ^ (t >> 16) ^ (t >> 32)) & 0xFFFFu);

	if ((s == 0u) || (s == old)) {
		s = (UH)(old + 0x9E37u);	/* step away; never 0 */
		if (s == 0u) {
			s = 1u;
		}
	}
	return s;
}

uint16_t tlm_bridge_begin_connect(uint8_t *p_will, uint16_t cap)
{
	g_session      = 0u;			/* not online until accepted */
	g_next_session = new_session(g_next_session);
	/* The will carries the new session, so the dashboard can tell which
	   connection went offline. */
	return (uint16_t)tlm_encode_online(g_next_session, false, p_will, cap);
}

uint16_t tlm_bridge_connected(uint8_t *p_online, uint16_t cap)
{
	/* Throw away frames queued while offline: they are stale. */
	while (tk_rcv_mbf(s1_mbf_out, g_rx, TMO_POL) > 0) {
		/* discard */
	}
	if (g_ever_online) {
		g_stats.reconnects++;
	}
	g_ever_online = TRUE;
	g_last_seq    = 0u;			/* new session: seq starts again */
	g_session     = g_next_session;		/* now commands are accepted */
	tm_printf((UB*)"tlm: online, session 0x%04x\n", (int)g_session);
	return (uint16_t)tlm_encode_online(g_session, true, p_online, cap);
}

void tlm_bridge_disconnected(void)
{
	if (g_session != 0u) {
		tm_printf((UB*)"tlm: offline\n");
	}
	g_session = 0u;
}

uint16_t tlm_bridge_next_frame(uint8_t *p_topic_id, uint8_t *p_buf, uint16_t cap)
{
	INT	n;
	INT	i;

	if (g_session == 0u) {
		return 0u;
	}
	n = tk_rcv_mbf(s1_mbf_out, g_rx, TMO_POL);
	if ((n < 2) || ((n - 1) > (INT)cap)) {
		return 0u;			/* empty, or not a valid frame */
	}
	*p_topic_id = g_rx[0];
	for (i = 1; i < n; i++) {
		p_buf[i - 1] = g_rx[i];
	}
	return (uint16_t)(n - 1);
}

const char *tlm_bridge_topic(uint8_t topic_id, uint8_t *p_qos, uint8_t *p_retain)
{
	*p_retain = 0u;
	switch (topic_id) {
	case TLM_TID_STATUS:	*p_qos = 0u; return TLM_TOPIC_STATUS;
	case TLM_TID_HEARTBEAT:	*p_qos = 0u; return TLM_TOPIC_HEARTBEAT;
	case TLM_TID_EVENT:	*p_qos = 1u; return TLM_TOPIC_EVENT;
	default:		*p_qos = 0u; return NULL;
	}
}

uint16_t tlm_bridge_handle_cmd(const uint8_t *p_data, uint16_t len,
			       uint8_t *p_ack, uint16_t cap)
{
	uint64_t	t0 = time_us_64();
	tlm_cmd_t	cmd;
	tlm_result_t	res;
	UH		last = g_last_seq;

	res = tlm_cmd_validate(p_data, len, g_session, &last, &cmd);

	if (res == TLM_RES_ACCEPTED) {
		g_last_seq = last;
		if (cmd.code == (uint8_t)TLM_CMD_ESTOP) {
			/* Urgent path: never waits behind queued events. */
			safety_raise_estop((uint8_t)EVT_SRC_REMOTE, (uint32_t)t0);
		} else {
			vehicle_event_t evt;

			evt.type    = (cmd.code == (uint8_t)TLM_CMD_START)?
				      (uint8_t)VEH_EVT_START: (uint8_t)VEH_EVT_STOP;
			evt.source  = (uint8_t)EVT_SRC_REMOTE;
			evt.arg     = cmd.seq;
			evt.t_rx_us = (uint32_t)t0;
			if (!vehicle_post_event(&evt)) {
				res = TLM_RES_QUEUE_FULL;	/* reported, never hidden */
				g_stats.queue_full++;
			}
		}
	}

	if (res == TLM_RES_ACCEPTED) {
		g_stats.cmds_ok++;
	} else {
		g_stats.cmds_rejected++;
	}
	g_stats.last_cmd_us = (UW)(time_us_64() - t0);

	tm_printf((UB*)"tlm: cmd code=%d seq=%d -> result=%d (%d us)\n",
		  (int)cmd.code, (int)cmd.seq, (int)res, (int)g_stats.last_cmd_us);

	return (uint16_t)tlm_encode_ack(g_session, g_ack_seq++, cmd.seq,
					cmd.code, (uint8_t)res, p_ack, cap);
}

/* ---- read-only views for our telemetry task (s1_internal.h) ---------- */

EXPORT UH tlm_bridge_session(void)
{
	return g_session;
}

EXPORT void tlm_bridge_get_stats(S1_STATS *p_out)
{
	p_out->cmds_ok       = g_stats.cmds_ok;
	p_out->cmds_rejected = g_stats.cmds_rejected;
	p_out->queue_full    = g_stats.queue_full;
	p_out->reconnects    = g_stats.reconnects;
	p_out->last_cmd_us   = g_stats.last_cmd_us;
}

EXPORT BOOL s1_is_online(void)
{
	return (g_session != 0u)? TRUE: FALSE;
}
