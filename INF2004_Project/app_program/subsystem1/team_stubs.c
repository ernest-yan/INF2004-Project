/*
 *	team_stubs.c
 *	Subsystem 1 -- FAKE versions of the other buddies' functions, so
 *	telemetry can be built and tested before their code exists.
 *
 *	Every function is "weak": as soon as a teammate adds the real
 *	function (same name) in their own folder, the linker uses theirs and
 *	ignores this one. Nothing to switch off by hand.
 *
 *	The fake qVehicleEvent holds 4 events and empties one every 500 ms,
 *	so sending commands quickly shows QUEUE_FULL.
 */

#include "../app.h"
#include "team_interfaces.h"

#define WEAK			__attribute__((weak))
#define STUB_QUEUE_LEN		4u
#define STUB_DRAIN_MS		500u

IMPORT uint64_t time_us_64(void);	/* pico-sdk timer, always running */

/* Written by the WiFi service task (commands), read by our telemetry task. */
LOCAL volatile UB	g_state = (UB)VEH_STATE_IDLE;
LOCAL volatile BOOL	g_estop = FALSE;
LOCAL UW		g_queue_count;
LOCAL UW		g_next_drain;

LOCAL UW now_ms(void)
{
	SYSTIM	t;

	tk_get_otm(&t);
	return (UW)t.lo;
}

WEAK bool vehicle_post_event(const vehicle_event_t *p_evt)
{
	UW	t = now_ms();

	/* pretend VehicleController takes one event out every 500 ms */
	while ((g_queue_count > 0u) && ((W)(t - g_next_drain) >= 0)) {
		g_queue_count--;
		g_next_drain += STUB_DRAIN_MS;
	}
	if (g_queue_count == 0u) {
		g_next_drain = t + STUB_DRAIN_MS;
	}
	if (g_queue_count >= STUB_QUEUE_LEN) {
		return false;		/* like tk_snd_mbf(..., TMO_POL) -> E_TMOUT */
	}
	g_queue_count++;
	tm_printf((UB*)"[STUB VC] queued event type=%d seq=%d (queue %d/%d)\n",
		  (int)p_evt->type, (int)p_evt->arg,
		  (int)g_queue_count, (int)STUB_QUEUE_LEN);

	if (!g_estop) {			/* a latched E-STOP ignores START */
		g_state = (p_evt->type == (uint8_t)VEH_EVT_START)?
			  (UB)VEH_STATE_LINE_FOLLOW: (UB)VEH_STATE_IDLE;
	}
	return true;
}

WEAK void safety_raise_estop(uint8_t source, uint32_t t_rx_us)
{
	g_estop = TRUE;
	g_state = (UB)VEH_STATE_SAFE_STOP;
	tm_printf((UB*)"[STUB VC] E-STOP latched, source=%d, %d us after rx\n",
		  (int)source, (int)((uint32_t)time_us_64() - t_rx_us));
}

WEAK void vehicle_get_status(vehicle_status_t *p_out)
{
	p_out->state           = g_state;
	p_out->b_estop_latched = g_estop;
	p_out->estop_source    = (uint8_t)EVT_SRC_REMOTE;
	p_out->fault_mask      = 0u;
}

WEAK void motion_get_status(motion_status_t *p_out)
{
	UW	t = now_ms();
	BOOL	moving = (g_state == (UB)VEH_STATE_LINE_FOLLOW);

	p_out->speed_l_mm_s = moving? (uint16_t)(300u + (t / 100u) % 20u): 0u;
	p_out->speed_r_mm_s = moving? (uint16_t)(300u + (t / 130u) % 20u): 0u;
	p_out->enc_l        = t / 10u;
	p_out->enc_r        = t / 10u + 7u;
	p_out->distance_mm  = t / 5u;
}

WEAK void line_get_status(line_status_t *p_out)
{
	UW	t = now_ms();

	p_out->line_state    = (uint8_t)((t / 3000u) % 4u);
	p_out->barcode_count = (uint16_t)(t / 20000u);	/* new barcode every 20 s */
	p_out->last_barcode  = (uint8_t)(1u + (p_out->barcode_count % 4u));
}

WEAK void imu_get_status(imu_status_t *p_out)
{
	UW	t = now_ms();
	UW	hump = t / 15000u;			/* peak rises every 15 s */

	p_out->pitch_deg    = (int8_t)((W)((t / 500u) % 21u) - 10);
	p_out->hump_max_mm  = (uint8_t)((hump > 40u)? 40u: hump);
	p_out->impact_count = 0u;
}

WEAK void ultra_get_status(ultra_status_t *p_out)
{
	UW	t = now_ms();

	p_out->obstacle_state = 0u;
	p_out->nearest_cm     = (uint16_t)(50u + (t / 200u) % 100u);
	p_out->profile_count  = (uint16_t)(t / 30000u);	/* profile every 30 s */
	p_out->centre_deg     = 90u;
	p_out->width_cm       = 12u;
	p_out->clear_l_cm     = 40u;
	p_out->clear_r_cm     = 25u;
	p_out->decision       = 2u;
}
