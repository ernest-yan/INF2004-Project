/*
 *	test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 *
 *	The profile analysis is a pure function, so it is tested here with fixed
 *	made-up scans -- no sensor or servo needed.
 */

#include "ultrasonic.h"
#include "../test_report.h"

#if APP_TEST

/* Build a scan: 5 coarse rays at 30..150 deg, 15 fine rays from fine_start
   in 5-degree steps. */
static void make_scan(subsystem5_scan_samples_t *s,
		      const uint16_t coarse_mm[5], uint8_t fine_start,
		      const uint16_t fine_mm[subsystem5_FINE_COUNT])
{
	int i;

	subsystem5_memset(s, 0, (int)sizeof(*s));
	for (i = 0; i < 5; i++) {
		s->coarse[i].angle_deg   = (uint16_t)(30 + 30 * i);
		s->coarse[i].distance_mm = coarse_mm[i];
		s->coarse[i].valid       = 1U;
	}
	for (i = 0; i < (int)subsystem5_FINE_COUNT; i++) {
		s->fine[i].angle_deg   = (uint16_t)(fine_start + 5 * i);
		s->fine[i].distance_mm = fine_mm[i];
		s->fine[i].valid       = 1U;
	}
	s->coarse_count = 5U;
	s->fine_count   = (uint8_t)subsystem5_FINE_COUNT;
}

/* Fine sweeps (55..125 deg) of a flat box face 300 mm ahead, simulated with
   a 15-degree HC-SR04 beam: the beam sees the box before the ray reaches its
   edge, which is what BEAM_EDGE_DEG corrects. Background at 900 mm. */
static const uint16_t box100[15] = { 900, 900, 900, 900, 303, 300, 300, 300,
				     300, 300, 303, 900, 900, 900, 900 };
static const uint16_t box200[15] = { 900, 900, 315, 307, 303, 300, 300, 300,
				     300, 300, 303, 307, 315, 900, 900 };
static const uint16_t box300[15] = { 900, 325, 315, 307, 303, 300, 300, 300,
				     300, 300, 303, 307, 315, 325, 900 };

static bool within_20pc(uint16_t got, uint16_t real)
{
	return ((uint32_t)got * 10U >= (uint32_t)real * 8U) &&
	       ((uint32_t)got * 10U <= (uint32_t)real * 12U);
}

EXPORT void subsystem5_test(void)
{
	subsystem5_scan_samples_t scan;
	subsystem5_profile_t p;
	subsystem5_status_t st;

	TEST_INIT();

	/* S5-U01: 200 mm box dead ahead, both sides open -> left (tie), and the
	   edge-based width is within 20 % (expected 215 mm). */
	{
		static const uint16_t c[5] = { 900, 900, 300, 900, 900 };
		make_scan(&scan, c, 55U, box200);
		p = subsystem5_profile_analyze(&scan, subsystem5_OBSTACLE_ZONE_MM,
					       subsystem5_SIDE_CLEAR_MM);
		TEST_CHECK("S5-U01",
			   p.obstacle_found == 1U && p.profile_valid == 1U &&
			   p.closest_mm == 300U &&
			   p.right_edge_deg == 65U && p.left_edge_deg == 115U &&
			   within_20pc(p.approximate_width_mm, 200U) &&
			   p.width_capped == 0U &&
			   p.candidate_side == subsystem5_SIDE_LEFT,
			   "200 mm box: edges 65/115 deg, width within 20 %, LEFT");
	}

	/* S5-U08: the SAME code measures other widths: 100 and 300 mm boxes */
	{
		static const uint16_t c1[5] = { 900, 900, 300, 900, 900 };
		static const uint16_t c3[5] = { 900, 325, 300, 325, 900 };
		subsystem5_profile_t q;
		make_scan(&scan, c1, 55U, box100);
		p = subsystem5_profile_analyze(&scan, subsystem5_OBSTACLE_ZONE_MM,
					       subsystem5_SIDE_CLEAR_MM);
		make_scan(&scan, c3, 55U, box300);
		q = subsystem5_profile_analyze(&scan, subsystem5_OBSTACLE_ZONE_MM,
					       subsystem5_SIDE_CLEAR_MM);
		TEST_CHECK("S5-U08",
			   within_20pc(p.approximate_width_mm, 100U) &&
			   within_20pc(q.approximate_width_mm, 300U) &&
			   p.candidate_side != subsystem5_SIDE_NONE &&
			   q.candidate_side != subsystem5_SIDE_NONE,
			   "100 mm and 300 mm boxes measured within 20 %");
	}

	/* S5-U02: left side ray blocked (300 mm), right open -> right */
	{
		static const uint16_t c[5] = { 900, 900, 300, 900, 300 };
		make_scan(&scan, c, 55U, box200);
		p = subsystem5_profile_analyze(&scan, subsystem5_OBSTACLE_ZONE_MM,
					       subsystem5_SIDE_CLEAR_MM);
		TEST_CHECK("S5-U02", p.profile_valid == 1U &&
			   p.candidate_side == subsystem5_SIDE_RIGHT,
			   "left blocked -> RIGHT");
	}

	/* S5-U03: both side rays blocked -> no safe side */
	{
		static const uint16_t c[5] = { 350, 900, 300, 900, 350 };
		make_scan(&scan, c, 55U, box200);
		p = subsystem5_profile_analyze(&scan, subsystem5_OBSTACLE_ZONE_MM,
					       subsystem5_SIDE_CLEAR_MM);
		TEST_CHECK("S5-U03", p.obstacle_found == 1U &&
			   p.candidate_side == subsystem5_SIDE_NONE,
			   "both sides blocked -> SIDE_NONE");
	}

	/* S5-U09: a wide box whose fine rays reach the side angles does not
	   block its own sides (sides come from the 30/150 rays only) */
	{
		static const uint16_t c[5] = { 900, 330, 300, 330, 900 };
		static const uint16_t wide[15] = { 330, 325, 315, 307, 303, 300,
						   300, 300, 300, 300, 303, 307,
						   315, 325, 330 };
		make_scan(&scan, c, 80U, wide);		/* sweep 80..150 */
		p = subsystem5_profile_analyze(&scan, subsystem5_OBSTACLE_ZONE_MM,
					       subsystem5_SIDE_CLEAR_MM);
		TEST_CHECK("S5-U09", p.candidate_side != subsystem5_SIDE_NONE &&
			   p.width_capped == 1U,
			   "own edges never block a side; width flagged as lower bound");
	}

	/* S5-U04: one invalid ray makes the profile invalid (never "open space") */
	{
		static const uint16_t c[5] = { 900, 900, 300, 900, 900 };
		make_scan(&scan, c, 55U, box200);
		scan.coarse[0].valid = 0U;
		p = subsystem5_profile_analyze(&scan, subsystem5_OBSTACLE_ZONE_MM,
					       subsystem5_SIDE_CLEAR_MM);
		TEST_CHECK("S5-U04", p.profile_valid == 0U,
			   "an invalid ray -> profile_valid == 0");
	}

	/* S5-U05: NULL or incomplete scan -> zeroed, invalid profile */
	p = subsystem5_profile_analyze(0, subsystem5_OBSTACLE_ZONE_MM,
				       subsystem5_SIDE_CLEAR_MM);
	subsystem5_memset(&scan, 0, (int)sizeof(scan));
	{
		subsystem5_profile_t q = subsystem5_profile_analyze(&scan,
				subsystem5_OBSTACLE_ZONE_MM, subsystem5_SIDE_CLEAR_MM);
		TEST_CHECK("S5-U05",
			   p.profile_valid == 0U && p.obstacle_found == 0U &&
			   q.profile_valid == 0U && q.obstacle_found == 0U,
			   "NULL / incomplete scan -> invalid, empty profile");
	}

	/* S5-U06: status snapshot readable while the poll task runs */
	subsystem5_get_status(&st);
	TEST_CHECK("S5-U06", st.state <= subsystem5_STATE_REVERSE,
		   "status snapshot is readable (mutex) and holds a valid state");

	/* S5-U07: event message buffer (IF-02) exists */
	{
		T_RMBF rmbf;
		TEST_CHECK("S5-U07", tk_ref_mbf(mbf_s5_events, &rmbf) == E_OK,
			   "mbf_s5_events (IF-02) is valid");
	}

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */