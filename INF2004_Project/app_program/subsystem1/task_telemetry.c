/*
 *	task_telemetry.c
 *	Subsystem 1 -- the telemetry task (ours, priority S1_PRI_TLM).
 *
 *	Every 200 ms: read every buddy's status (their *_get_status()
 *	getters), pack one STATUS frame, and drop it into s1_mbf_out.
 *	Every 1 s:    pack one HEARTBEAT frame.
 *	On a change:  pack an EVENT frame (barcode, hump peak, obstacle,
 *	              impact, state change, E-STOP).
 *	On a command: act on what the WiFi task queued in s1_mbf_cmd.
 *
 *	It never touches WiFi/lwIP/MQTT: the WiFi service task empties the
 *	message buffer (tlm_bridge.c + mqtt_utk.c). It never waits on the
 *	buffer either (TMO_POL): if it is full, the frame is dropped and
 *	counted, and the car carries on.
 */

#include "subsystem1.h"
#include "../app_hw.h"
#include "s1_internal.h"
#include "team_interfaces.h"
#include "tlm_codec.h"

#define MS_PER_S	1000u

LOCAL volatile S1_DIAG	g_diag;		/* read by our tests */
LOCAL UW	g_tx_dropped;		/* frames lost because the buffer was full */
LOCAL UB	g_seq_status;
LOCAL UB	g_seq_heartbeat;
LOCAL UB	g_seq_event;

/* Previous snapshot, to spot changes (events). */
LOCAL BOOL		g_prev_valid;
LOCAL vehicle_status_t	g_prev_v;
LOCAL line_status_t	g_prev_l;
LOCAL imu_status_t	g_prev_i;
LOCAL ultra_status_t	g_prev_u;

LOCAL UW now_ms(void)
{
	SYSTIM	t;

	tk_get_otm(&t);			/* time since power-on */
	return (UW)t.lo;
}

/* Queue one frame for the WiFi task. Never waits. */
LOCAL void send_frame(UB topic_id, const UB *p_payload, UW len)
{
	UB	msg[TLM_MBF_MSG_MAX];
	UW	i;
	ER	er;

	if ((len == 0u) || (len > TLM_FRAME_MAX)) {
		return;
	}
	msg[0] = topic_id;
	for (i = 0u; i < len; i++) {
		msg[i + 1u] = p_payload[i];
	}
	er = tk_snd_mbf(s1_mbf_out, msg, (INT)(len + 1u), TMO_POL);
	if (er != E_OK) {
		g_tx_dropped++;			/* E_TMOUT: buffer full */
	}
}

LOCAL void send_event(tlm_event_t *p_e, UH session)
{
	UB	buf[TLM_EVENT_MAX_LEN];
	UW	n;

	p_e->session = session;
	p_e->seq     = g_seq_event++;
	n = tlm_encode_event(p_e, buf, sizeof(buf));
	send_frame(TLM_TID_EVENT, buf, n);
}

/* Compare with the previous snapshot; one EVENT per change. */
LOCAL void detect_events(UH session, const vehicle_status_t *p_v,
			 const line_status_t *p_l, const imu_status_t *p_i,
			 const ultra_status_t *p_u)
{
	tlm_event_t	e = {0};

	if (!g_prev_valid) {
		return;
	}
	if (p_v->state != g_prev_v.state) {
		e.type = (uint8_t)TLM_EVT_STATE_CHANGE;
		e.a    = g_prev_v.state;
		e.b    = p_v->state;
		send_event(&e, session);
	}
	if (p_v->b_estop_latched && !g_prev_v.b_estop_latched) {
		tlm_event_t x = {0};
		x.type = (uint8_t)TLM_EVT_ESTOP;
		x.a    = p_v->estop_source;
		send_event(&x, session);
	}
	if (p_l->barcode_count != g_prev_l.barcode_count) {
		tlm_event_t x = {0};
		x.type = (uint8_t)TLM_EVT_BARCODE;
		x.a    = p_l->last_barcode;
		send_event(&x, session);
	}
	if (p_i->hump_max_mm > g_prev_i.hump_max_mm) {
		tlm_event_t x = {0};
		x.type = (uint8_t)TLM_EVT_HUMP_PEAK;
		x.a    = p_i->hump_max_mm;
		send_event(&x, session);
	}
	if (p_i->impact_count != g_prev_i.impact_count) {
		tlm_event_t x = {0};
		x.type = (uint8_t)TLM_EVT_IMPACT;
		send_event(&x, session);
	}
	if (p_u->profile_count != g_prev_u.profile_count) {
		tlm_event_t x = {0};
		x.type           = (uint8_t)TLM_EVT_OBSTACLE;
		x.obs_nearest_cm = p_u->nearest_cm;
		x.obs_centre_deg = p_u->centre_deg;
		x.obs_width_cm   = p_u->width_cm;
		x.obs_clear_l_cm = p_u->clear_l_cm;
		x.obs_clear_r_cm = p_u->clear_r_cm;
		x.obs_decision   = p_u->decision;
		send_event(&x, session);
	}
}

LOCAL void send_status(UH session)
{
	vehicle_status_t	v;
	motion_status_t		m;
	line_status_t		l;
	imu_status_t		i;
	ultra_status_t		u;
	tlm_status_t		s = {0};
	UW			dist_cm;

	/* Each getter copies its owner's data under the owner's own lock. */
	vehicle_get_status(&v);
	motion_get_status(&m);
	line_get_status(&l);
	imu_get_status(&i);
	ultra_get_status(&u);

	dist_cm = m.distance_mm / 10u;
	s.session         = session;
	s.seq             = g_seq_status++;
	s.vehicle_state   = v.state;
	s.b_estop_latched = v.b_estop_latched;
	s.line_state      = l.line_state;
	s.last_barcode    = l.last_barcode;
	s.speed_l_mm_s    = m.speed_l_mm_s;
	s.speed_r_mm_s    = m.speed_r_mm_s;
	s.enc_l           = (uint16_t)(m.enc_l & 0xFFFFu);	/* wraps */
	s.enc_r           = (uint16_t)(m.enc_r & 0xFFFFu);
	s.distance_cm     = (uint16_t)((dist_cm > 0xFFFFu)? 0xFFFFu: dist_cm);
	s.pitch_deg       = i.pitch_deg;
	s.hump_max_mm     = i.hump_max_mm;
	s.obstacle_state  = u.obstacle_state;
	s.obstacle_cm     = u.nearest_cm;
	s.fault_mask      = v.fault_mask;

#if TLM_USE_JSON_STATUS
	{
		char	json[TLM_FRAME_MAX];
		UW	n = tlm_format_status_json(&s, json, sizeof(json));
		send_frame(TLM_TID_STATUS, (const UB*)json, n);
	}
#else
	{
		UB	buf[TLM_STATUS_LEN];
		UW	n = tlm_encode_status(&s, buf, sizeof(buf));
		send_frame(TLM_TID_STATUS, buf, n);
	}
#endif

	detect_events(session, &v, &l, &i, &u);
	g_prev_v = v;
	g_prev_l = l;
	g_prev_i = i;
	g_prev_u = u;
	g_prev_valid = TRUE;
}

/* Act on the commands the WiFi task accepted (s1_mbf_cmd). Runs here,
 * not in the WiFi task, so the calls below may lock other subsystems. */
LOCAL void run_commands(void)
{
	S1_CMD	q;

	while (tk_rcv_mbf(s1_mbf_cmd, &q, TMO_POL) == (INT)sizeof(q)) {
		vehicle_event_t evt;

		switch (q.code) {
		case TLM_CMD_ESTOP:
			safety_raise_estop((uint8_t)EVT_SRC_REMOTE, q.t_rx_us);
			break;
		case TLM_CMD_RESET:
			(void)safety_reset_while_stopped();
			break;
		case TLM_CMD_MAP:
			barcode_set_mapping(q.map);
			break;
		default:			/* START / STOP */
			evt.type    = (q.code == (UB)TLM_CMD_START)?
				      (uint8_t)VEH_EVT_START: (uint8_t)VEH_EVT_STOP;
			evt.source  = (uint8_t)EVT_SRC_REMOTE;
			evt.arg     = q.seq;
			evt.t_rx_us = q.t_rx_us;
			(void)vehicle_post_event(&evt);
			break;
		}
	}
}

LOCAL void send_heartbeat(UH session, UW now)
{
	S1_STATS	st;
	tlm_heartbeat_t	h = {0};
	UB		buf[TLM_HEARTBEAT_LEN];
	UW		up_s = now / MS_PER_S;
	UW		n;

	tlm_bridge_get_stats(&st);
	h.session       = session;
	h.seq           = g_seq_heartbeat++;
	h.uptime_s      = (uint16_t)((up_s > 0xFFFFu)? 0xFFFFu: up_s);
	h.rssi_neg_dbm  = 0u;		/* not available to app tasks: 0 = n/a */
	h.link_state    = 3u;		/* we only send while ONLINE */
	h.cmds_ok       = (uint8_t)st.cmds_ok;
	h.cmds_rejected = (uint8_t)st.cmds_rejected;
	h.queue_full    = (uint8_t)st.queue_full;
	h.reconnects    = (uint8_t)st.reconnects;

	n = tlm_encode_heartbeat(&h, buf, sizeof(buf));
	send_frame(TLM_TID_HEARTBEAT, buf, n);
}

EXPORT void s1_telemetry_task(INT stacd, void *exinf)
{
	UW	next_status    = 0u;
	UW	next_heartbeat = 0u;

	(void)stacd;
	(void)exinf;
	tm_printf((UB*)"tlm: telemetry task started (pri %d)\n", S1_PRI_TLM);

	for (;;) {
		uint64_t t_start = app_time_us_64();
		UW	now     = now_ms();
		UH	session = tlm_bridge_session();
		UW	took_us;

		run_commands();
		if (session == 0u) {
			g_prev_valid = FALSE;	/* offline: nothing to send */
		} else {
			if ((W)(now - next_status) >= 0) {
				next_status = now + TLM_STATUS_PERIOD_MS;
				send_status(session);
			}
			if ((W)(now - next_heartbeat) >= 0) {
				next_heartbeat = now + TLM_HEARTBEAT_PERIOD_MS;
				send_heartbeat(session, now);
			}
		}

		took_us = (UW)(app_time_us_64() - t_start);
		if (took_us > g_diag.loop_max_us) {
			g_diag.loop_max_us = took_us;
		}
		g_diag.cores_seen |= 1u << tk_get_prc();
		g_diag.loops++;

		/* Sleep one tick, or less if the WiFi task wakes us with a
		   command (tk_wup_tsk in tlm_bridge.c). */
		tk_slp_tsk(TLM_TASK_TICK_MS);
	}
}

EXPORT void s1_tlm_get_diag(S1_DIAG *p_out)
{
	p_out->loops       = g_diag.loops;
	p_out->loop_max_us = g_diag.loop_max_us;
	p_out->cores_seen  = g_diag.cores_seen;
}
