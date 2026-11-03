/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4 (accelerometer).  Built only with: make TEST=1
 *	Record results in testing/subsystem4/TEST_LOG.md.  IDs: S4-U01, ...
 *
 *	S4-U01/U02 need the GY-511 wired to GP26/GP27 and the robot kept still.
 *	S4-U03/U04 test the hump-profile maths with made-up data (no sensor).
 */

#include "tkernel_adapter.h"
#include "terrain.h"
#include "../test_report.h"

#if APP_TEST

/* Feed n samples of the same pitch, step_mm of travel each */
static bool feed(b4_terrain_t *m, float pitch, uint32_t step_mm, int n)
{
	bool done = false;
	int i;

	for (i = 0; i < n; i++) {
		done |= b4_terrain_update(m, pitch, step_mm);
	}
	return done;
}

EXPORT void subsystem4_test(void)
{
	b4_status_t st;
	b4_terrain_t m;

	TEST_INIT();

	/* S4-U01: sensor found and configured on I2C1 */
	tk_dly_tsk(200);
	TEST_CHECK("S4-U01", s4_ready(), "LSM303DLHC answers on I2C1 GP26/GP27");

	/* S4-U02: calibration finishes (50 samples x 10 ms) and gravity is ~1 g */
	tk_dly_tsk(1000);
	b4_get_status(&st);
	{
		INT ax = st.acceleration_x_mg, ay = st.acceleration_y_mg;
		INT az = st.acceleration_z_mg;
		INT g2 = ax * ax + ay * ay + az * az;	/* mg^2 */
		TEST_CHECK("S4-U02", st.calibration_ready == 1U &&
			   st.state == B4_STATE_MONITOR &&
			   g2 > 700 * 700 && g2 < 1300 * 1300,
			   "calibrated, |a| between 0.7 g and 1.3 g at rest");
	}

	/* S4-U03: 10 x 10 mm at +10 deg, then down, then level = one hump.
	   The first 2 climb samples only arm the detector, so the expected
	   height is 8 x 10 mm x sin(10 deg) = 13.9 mm -> 14 mm. */
	b4_terrain_init(&m);
	{
		bool done = feed(&m, 10.0f, 10U, 10);	/* climb */
		done |= feed(&m, -10.0f, 10U, 10);	/* descend */
		done |= feed(&m, 0.0f, 10U, 3);		/* level again */
		TEST_CHECK("S4-U03", done && m.last_peak_mm >= 13U &&
			   m.last_peak_mm <= 15U,
			   "hump detected, height 13-15 mm");
	}

	/* S4-U04: flat ground never reports a hump */
	b4_terrain_init(&m);
	TEST_CHECK("S4-U04", !feed(&m, 1.0f, 10U, 200) && m.max_peak_mm == 0U,
		   "level travel reports no hump");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
