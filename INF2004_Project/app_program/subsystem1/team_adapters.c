/*
 *	team_adapters.c
 *	Subsystem 1 -- turns the other buddies' REAL status into the
 *	telemetry structs in team_interfaces.h.
 *
 *	Only CALLS their public getters; never changes their code or data.
 *	Called only from OUR telemetry task (never from the WiFi task),
 *	because some getters take a kernel mutex and may wait briefly.
 *
 *	  motion_get_status()  <- b2_get_status()               (Buddy 2)
 *	  line_get_status()    <- get_line_barcode_data_safe()  (Buddy 3)
 *	  imu_get_status()     <- b4_get_status()               (Buddy 4)
 *	  ultra_get_status()   <- subsystem5_get_status()       (Buddy 5)
 *	  vehicle_get_status() <- worked out from all of the above
 *	                          (there is no VehicleController state yet)
 */

#include "../app.h"
#include "team_interfaces.h"
#include "../subsystem2/tkernel_adapter.h"
#include "../subsystem3/line_barcode.h"
#include "../subsystem4/imu.h"
#include "../subsystem5/ultrasonic.h"

#define MM_PER_CM		10
#define CDEG_PER_DEG		100
#define PITCH_MIN_DEG		(-64)		/* 7-bit field in the STATUS frame */
#define PITCH_MAX_DEG		63
#define NO_ECHO_CM		0x1FFu		/* 9-bit max: no valid reading */

/* Rising-edge counters for the telemetry EVENT frames. */
LOCAL BOOL	g_prev_impact;
LOCAL UH	g_impact_count;
LOCAL BOOL	g_prev_scan;
LOCAL UH	g_profile_count;

LOCAL uint16_t clamp_u16(int32_t v)
{
	if (v < 0) {
		v = -v;			/* speed: direction is not sent */
	}
	return (uint16_t)((v > 0xFFFF)? 0xFFFF: v);
}

LOCAL uint8_t clamp_u8(uint32_t v)
{
	return (uint8_t)((v > 0xFFu)? 0xFFu: v);
}

/* ---- remote commands (from our telemetry task, never the WiFi task) ---- */

/* PENDING TEAM DECISION: what remote START / STOP should do. There is no
 * VehicleController queue yet, so the command is only logged. */
bool vehicle_post_event(const vehicle_event_t *p_evt)
{
	tm_printf((UB*)"[S1] remote %s (seq %d): no action yet, team to decide\n",
		  (p_evt->type == (uint8_t)VEH_EVT_START)? "START": "STOP",
		  (int)p_evt->arg);
	return true;
}

/* PENDING TEAM DECISION: remote E-STOP is NOT wired to the brakes yet.
 * Suggested: b2_emergency_brake() (latches until the Pico is reset). */
void safety_raise_estop(uint8_t source, uint32_t t_rx_us)
{
	(void)source;
	(void)t_rx_us;
	tm_printf((UB*)"[S1] remote E-STOP: NOT wired to the brakes yet, team to decide\n");
}

/* ---- status ------------------------------------------------------------ */

void vehicle_get_status(vehicle_status_t *p_out)
{
	b2_status_t		m;
	b4_status_t		i;
	subsystem5_status_t	u;
	LineBarcodeData_t	l;
	uint8_t			faults = 0u;

	b2_get_status(&m);
	b4_get_status(&i);
	subsystem5_get_status(&u);
	l = get_line_barcode_data_safe();

	/* Most serious first: the first match wins. */
	if (!s2_ready()) {
		p_out->state = (uint8_t)VEH_STATE_INIT;
	} else if (m.brake_latched || (m.state == B2_FAULT) ||
		   i.impact_latched || (u.state == subsystem5_STATE_FAULT)) {
		p_out->state = (uint8_t)VEH_STATE_SAFE_STOP;
	} else if (u.state != subsystem5_STATE_MONITOR) {
		p_out->state = (uint8_t)VEH_STATE_OBSTACLE_AVOID;
	} else if (m.state == B2_EXECUTE_GOAL) {
		/* S5's bypass goals are caught above, so a goal here is S3's
		   barcode turn or junction advance. */
		p_out->state = (uint8_t)VEH_STATE_BARCODE_NAV;
	} else if (m.state == B2_TRACK_SPEED) {
		p_out->state = l.line_detected? (uint8_t)VEH_STATE_LINE_FOLLOW:
						(uint8_t)VEH_STATE_LINE_SEARCH;
	} else {
		p_out->state = (uint8_t)VEH_STATE_IDLE;
	}

	/* The only latch is S2's brake (S4 pulls it on an impact). */
	p_out->b_estop_latched = (m.brake_latched != 0u);
	p_out->estop_source    = i.impact_latched? (uint8_t)EVT_SRC_IMU:
						(uint8_t)EVT_SRC_MOTION;

	if ((m.fault != B2_FAULT_NONE) || (m.state == B2_FAULT)) {
		faults |= TLM_FAULT_S2;
	}
	if ((l.fault_mask & (uint8_t)~FAULT_LINE_LOSS) != 0u) {
		faults |= TLM_FAULT_S3;	/* line loss is not a fault: see line */
	}
	if (i.fault != B4_FAULT_NONE) {
		faults |= TLM_FAULT_S4;
	}
	if (u.fault != subsystem5_FAULT_NONE) {
		faults |= TLM_FAULT_S5;
	}
	p_out->fault_mask = faults;
}

void motion_get_status(motion_status_t *p_out)
{
	b2_status_t	s;
	int32_t		mean_mm;

	b2_get_status(&s);
	mean_mm = (s.left_total_mm + s.right_total_mm) / 2;

	p_out->speed_l_mm_s = clamp_u16(s.left_speed_mm_s);
	p_out->speed_r_mm_s = clamp_u16(s.right_speed_mm_s);
	p_out->enc_l        = s.left_encoder_edges;
	p_out->enc_r        = s.right_encoder_edges;
	p_out->distance_mm  = (mean_mm > 0)? (uint32_t)mean_mm: 0u;
}

void line_get_status(line_status_t *p_out)
{
	LineBarcodeData_t	d = get_line_barcode_data_safe();

	/* Sensors are active-high (1 = on the line). */
	p_out->line_state = (uint8_t)((d.ir_left_raw << 1) | d.ir_center_raw);

	/* TODO(Buddy 3): S3 keeps a decoded letter for only one poll (a few
	 * microseconds) and its queue belongs to line_follow_task, so we
	 * cannot see barcodes reliably. Needs a counter + last letter in
	 * LineBarcodeData_t. Until then: no barcode events. */
	p_out->last_barcode  = (uint8_t)BARCODE_NONE;
	p_out->barcode_count = 0u;
}

void imu_get_status(imu_status_t *p_out)
{
	b4_status_t	s;
	W		pitch;

	b4_get_status(&s);
	pitch = (W)s.pitch_cdeg / CDEG_PER_DEG;
	if (pitch < PITCH_MIN_DEG) {
		pitch = PITCH_MIN_DEG;
	} else if (pitch > PITCH_MAX_DEG) {
		pitch = PITCH_MAX_DEG;
	}

	/* S4 only latches an impact: count each new latch. */
	if (s.impact_latched && !g_prev_impact) {
		g_impact_count++;
	}
	g_prev_impact = (s.impact_latched != 0u);

	p_out->pitch_deg    = (int8_t)pitch;
	p_out->hump_max_mm  = clamp_u8(s.max_hump_height_mm);
	p_out->impact_count = g_impact_count;
}

void ultra_get_status(ultra_status_t *p_out)
{
	subsystem5_status_t		s;
	const subsystem5_profile_t	*p = &s.profile;

	subsystem5_get_status(&s);

	/* One OBSTACLE event per finished scan. */
	if (s.scan_complete && !g_prev_scan) {
		g_profile_count++;
	}
	g_prev_scan = (s.scan_complete != 0u);

	p_out->obstacle_state = (uint8_t)s.state;
	p_out->nearest_cm     = s.front_valid? (uint16_t)(s.front_mm / MM_PER_CM):
					       (uint16_t)NO_ECHO_CM;
	p_out->profile_count  = g_profile_count;
	p_out->centre_deg     = clamp_u8(p->closest_angle_deg);
	p_out->width_cm       = clamp_u8(p->approximate_width_mm / MM_PER_CM);
	p_out->clear_l_cm     = p->left_clearance_valid?
				(uint16_t)(p->left_clearance_mm / MM_PER_CM): 0u;
	p_out->clear_r_cm     = p->right_clearance_valid?
				(uint16_t)(p->right_clearance_mm / MM_PER_CM): 0u;
	p_out->decision       = (uint8_t)p->candidate_side;
}
