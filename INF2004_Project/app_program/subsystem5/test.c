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

/* Build a scan: 5 coarse rays at 30..150 deg, 5 fine rays at the given angles */
static void make_scan(subsystem5_scan_samples_t *s,
		      const uint16_t coarse_mm[5],
		      const uint8_t fine_deg[5],
		      const uint16_t fine_mm[5])
{
	int i;

	subsystem5_memset(s, 0, (int)sizeof(*s));
	for (i = 0; i < 5; i++) {
		s->coarse[i].angle_deg   = (uint16_t)(30 + 30 * i);
		s->coarse[i].distance_mm = coarse_mm[i];
		s->coarse[i].valid       = 1U;
		s->fine[i].angle_deg     = fine_deg[i];
		s->fine[i].distance_mm   = fine_mm[i];
		s->fine[i].valid         = 1U;
	}
	s->coarse_count = 5U;
	s->fine_count   = 5U;
}

EXPORT void subsystem5_test(void)
{
	static const uint8_t  fdeg[5] = { 70, 80, 90, 100, 110 };
	subsystem5_scan_samples_t scan;
	subsystem5_profile_t p;
	subsystem5_status_t st;

	TEST_INIT();

	/* S5-U01: obstacle dead ahead, both sides open -> left (the wider side) */
	{
		static const uint16_t c[5] = { 900, 900, 300, 900, 900 };
		static const uint16_t f[5] = { 900, 320, 300, 320, 900 };
		make_scan(&scan, c, fdeg, f);
		p = subsystem5_profile_analyze(&scan, subsystem5_OBSTACLE_ZONE_MM,
					       subsystem5_SIDE_CLEAR_MM);
		TEST_CHECK("S5-U01",
			   p.obstacle_found == 1U && p.profile_valid == 1U &&
			   p.closest_mm == 300U && p.closest_angle_deg == 90U &&
			   p.span_deg == 20U && p.approximate_width_mm == 104U &&
			   p.candidate_side == subsystem5_SIDE_LEFT,
			   "obstacle ahead, both sides open -> LEFT, width 104 mm");
	}

	/* S5-U02: left side blocked, right open -> right */
	{
		static const uint16_t c[5] = { 900, 900, 300, 400, 450 };
		static const uint16_t f[5] = { 900, 320, 300, 320, 450 };
		make_scan(&scan, c, fdeg, f);
		p = subsystem5_profile_analyze(&scan, subsystem5_OBSTACLE_ZONE_MM,
					       subsystem5_SIDE_CLEAR_MM);
		TEST_CHECK("S5-U02", p.profile_valid == 1U &&
			   p.candidate_side == subsystem5_SIDE_RIGHT,
			   "left blocked -> RIGHT");
	}

	/* S5-U03: both sides blocked -> no safe side */
	{
		static const uint16_t c[5] = { 400, 400, 300, 400, 400 };
		static const uint16_t f[5] = { 400, 400, 300, 400, 400 };
		make_scan(&scan, c, fdeg, f);
		p = subsystem5_profile_analyze(&scan, subsystem5_OBSTACLE_ZONE_MM,
					       subsystem5_SIDE_CLEAR_MM);
		TEST_CHECK("S5-U03", p.obstacle_found == 1U &&
			   p.candidate_side == subsystem5_SIDE_NONE,
			   "both sides blocked -> SIDE_NONE");
	}

	/* S5-U04: one invalid ray makes the profile invalid (never "open space") */
	{
		static const uint16_t c[5] = { 900, 900, 300, 900, 900 };
		static const uint16_t f[5] = { 900, 320, 300, 320, 900 };
		make_scan(&scan, c, fdeg, f);
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
	TEST_CHECK("S5-U06", st.state <= subsystem5_STATE_FAULT,
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