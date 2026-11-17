/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4 (accelerometer).  Built only with: make TEST=1
 *	Record results in testing/subsystem4/TEST_LOG.md.  IDs: S4-U01, ...
 *
 *	S4-U01/U02 need the GY-511 wired to GP26/GP27 and the robot kept still.
 *	S4-U03/U04 test the hump-profile maths with made-up data (no sensor).
 *	S4-U05..U08 test lift / overturn detection with made-up data.
 *	S4-U09/U10 test turn rate and motion classification.
 */

#include "tkernel_adapter.h"
#include "terrain.h"
#include "posture.h"
#include "motion_class.h"
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

/* Feed n identical accelerometer samples; return the last result and the
   sample number (1-based) of the first non-NONE result, 0 if none. */
static b4_lift_cause_t feed_acc(b4_posture_t *p, const b4_posture_cfg_t *c,
				float ax, float ay, float az, int n, int *p_first)
{
	b4_lift_cause_t r = B4_LIFT_NONE;
	int i;

	for (i = 0; i < n; i++) {
		r = b4_posture_update(p, c, ax, ay, az);
		if ((r != B4_LIFT_NONE) && (p_first != NULL) && (*p_first == 0)) {
			*p_first = i + 1;
		}
	}
	return r;
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

	/* Same thresholds as subsystem4_init(). Level robot: gravity on +Z. */
	{
		const b4_posture_cfg_t c = {250U, 150U, 30U, 60U};
		b4_posture_t p;
		b4_lift_cause_t r;
		int first;

		/* S4-U05: 15 deg humps, heavy vibration and a hump-edge
		   shock: no false lift. sin/cos(15 deg) = 0.259 / 0.966. */
		b4_posture_init(&p, 0.0f, 0.0f, 1000.0f);
		first = 0;
		(void)feed_acc(&p, &c, -259.0f, 0.0f, 966.0f, 50, &first);
		(void)feed_acc(&p, &c, 259.0f, 0.0f, 966.0f, 50, &first);
		/* 2 s of heavy driving vibration: +-300 mg every sample */
		{
			int k;
			for (k = 0; k < 100; k++) {
				(void)feed_acc(&p, &c, 150.0f, 0.0f, 1300.0f, 1, &first);
				(void)feed_acc(&p, &c, -150.0f, 0.0f, 700.0f, 1, &first);
			}
		}
		/* hump edge: kicked up at 1.6 g for 30 ms, then it falls
		   back (0.4 g for 30 ms): what goes up comes down */
		(void)feed_acc(&p, &c, 0.0f, 0.0f, 1600.0f, 3, &first);
		(void)feed_acc(&p, &c, 0.0f, 0.0f, 400.0f, 3, &first);
		r = feed_acc(&p, &c, 0.0f, 0.0f, 1000.0f, 20, &first);
		TEST_CHECK("S4-U05", (r == B4_LIFT_NONE) && (first == 0),
			   "15 deg humps, vibration, edge shock: no lift");

		/* S4-U06: picked straight up (1.4 g) -> LIFTED within 100 ms */
		b4_posture_init(&p, 0.0f, 0.0f, 1000.0f);
		first = 0;
		r = feed_acc(&p, &c, 0.0f, 0.0f, 1400.0f, 10, &first);
		TEST_CHECK("S4-U06", (r == B4_LIFT_LIFTED) && (first > 0) &&
			   (first <= 10), "upward 0.4 g held -> LIFTED <= 100 ms");

		/* S4-U07: held tilted 40 deg (carried) -> LIFTED within 500 ms */
		b4_posture_init(&p, 0.0f, 0.0f, 1000.0f);
		first = 0;
		r = feed_acc(&p, &c, 643.0f, 0.0f, 766.0f, 50, &first);
		TEST_CHECK("S4-U07", (r == B4_LIFT_LIFTED) && (first > 0),
			   "held at 40 deg tilt -> LIFTED");

		/* S4-U08: upside down -> OVERTURNED */
		b4_posture_init(&p, 0.0f, 0.0f, 1000.0f);
		r = feed_acc(&p, &c, 0.0f, 0.0f, -1000.0f, 50, NULL);
		TEST_CHECK("S4-U08", (r == B4_LIFT_OVERTURNED) &&
			   (p.tilt_deg > 170.0f), "upside down -> OVERTURNED");
	}

	/* S4-U11: gentle pick-up while driving: 20 cm in 1.5 s, peak only
	   57 mg (far below lift_accel_mg), on top of +-200 mg vibration.
	   a(t) = 57 mg * sin(2 pi t / 1.5 s), 150 samples at 10 ms.
	   sin() is a parabola approximation: good enough for a test. */
	{
		const b4_posture_cfg_t c = {250U, 150U, 30U, 60U};
		b4_posture_t p;
		b4_lift_cause_t r = B4_LIFT_NONE;
		int i, first = 0;

		b4_posture_init(&p, 0.0f, 0.0f, 1000.0f);
		for (i = 0; (i < 150) && (first == 0); i++) {
			const float x = (float)i / 75.0f;	/* 0..2 */
			const float s = (x < 1.0f) ? (4.0f * x * (1.0f - x))
					: (-4.0f * (x - 1.0f) * (2.0f - x));
			const float vib = ((i & 1) != 0) ? 200.0f : -200.0f;
			r = b4_posture_update(&p, &c, vib, 0.0f,
					      1000.0f + (57.0f * s) + vib);
			if (r != B4_LIFT_NONE) { first = i + 1; }
		}
		TEST_CHECK("S4-U11", (r == B4_LIFT_LIFTED) && (first > 0) &&
			   (first <= 75),
			   "gentle 20 cm / 1.5 s lift in vibration -> LIFTED <= 0.75 s");
	}

	/* S4-U12: put DOWN from 20 cm in 1 s, landing with a 0.5 g thump for
	   40 ms: must NOT read as a lift. Same parabola "sine" as S4-U11,
	   upside down (down first, then the stop). */
	{
		const b4_posture_cfg_t c = {250U, 150U, 30U, 60U};
		b4_posture_t p;
		int i, first = 0;

		b4_posture_init(&p, 0.0f, 0.0f, 1000.0f);
		for (i = 0; i < 100; i++) {
			const float x = (float)i / 50.0f;	/* 0..2 */
			const float s = (x < 1.0f) ? (4.0f * x * (1.0f - x))
					: (-4.0f * (x - 1.0f) * (2.0f - x));
			if (b4_posture_update(&p, &c, 0.0f, 0.0f,
					      1000.0f - (128.0f * s))
			    != B4_LIFT_NONE) { first = i + 1; }
		}
		(void)feed_acc(&p, &c, 0.0f, 0.0f, 1500.0f, 4, &first);
		(void)feed_acc(&p, &c, 0.0f, 0.0f, 1000.0f, 150, &first);
		TEST_CHECK("S4-U12", first == 0,
			   "put down with a landing thump: no lift");
	}

	/* S4-U09: turn rate from wheel speeds, track 120 mm.
	   Pivot left at 100 mm/s: (100 - -100) / 120 rad/s = 95.5 deg/s. */
	{
		const float yaw = b4_yaw_rate_dps(-100, 100, 120U);
		TEST_CHECK("S4-U09", (yaw > 94.5f) && (yaw < 96.5f) &&
			   (b4_yaw_rate_dps(150, 150, 120U) == 0.0f),
			   "pivot left 95.5 deg/s, straight 0 deg/s");
	}

	/* S4-U10: motion classes, most important first */
	TEST_CHECK("S4-U10",
		   (b4_classify_motion(false, B4_LIFT_NONE, B4_TERRAIN_LEVEL,
				       0, 0, 0.0f) == B4_MOTION_CALIBRATING) &&
		   (b4_classify_motion(true, B4_LIFT_NONE, B4_TERRAIN_LEVEL,
				       0, 0, 0.0f) == B4_MOTION_STATIONARY) &&
		   (b4_classify_motion(true, B4_LIFT_NONE, B4_TERRAIN_LEVEL,
				       150, 150, 0.0f) == B4_MOTION_MOVING) &&
		   (b4_classify_motion(true, B4_LIFT_NONE, B4_TERRAIN_LEVEL,
				       -100, 100, 95.5f) == B4_MOTION_TURNING) &&
		   (b4_classify_motion(true, B4_LIFT_NONE, B4_TERRAIN_CLIMB,
				       150, 150, 0.0f) == B4_MOTION_ON_HUMP) &&
		   (b4_classify_motion(true, B4_LIFT_LIFTED, B4_TERRAIN_CLIMB,
				       150, 150, 0.0f) == B4_MOTION_LIFTED) &&
		   (b4_classify_motion(true, B4_LIFT_OVERTURNED, B4_TERRAIN_LEVEL,
				       0, 0, 0.0f) == B4_MOTION_OVERTURNED),
		   "stationary / moving / turning / hump / lifted / overturned");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
