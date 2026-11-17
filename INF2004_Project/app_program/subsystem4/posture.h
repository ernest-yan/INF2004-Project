/*
 *	posture.h
 *	Subsystem 4 -- "has the robot been lifted or overturned?"
 *
 *	Pure maths, no hardware: imu.c feeds it one accelerometer sample
 *	(car frame, mg) every 10 ms. Tested with made-up data in
 *	subsystem4_test.c (S4-U05..U08).
 *
 *	How it decides (all thresholds PROVISIONAL, tune on the robot):
 *	  OVERTURNED  the gravity vector has turned more than overturn_tilt_deg
 *	              away from the calibrated "level" direction for 100 ms.
 *	  LIFTED      any one of:
 *	              - it is MOVING upwards faster than lift_speed_mm_s for
 *	                100 ms. Upward speed = vertical acceleration integrated
 *	                over about 1 s. This is the rule that catches a normal
 *	                or gentle pick-up while driving: vibration is zero on
 *	                average so it cancels in the integral, and a hump at
 *	                line speed only lifts the body at ~20 mm/s.
 *	              - a sharp upward jolt: more than lift_accel_mg above 1 g
 *	                for 50 ms (a quick snatch);
 *	              - it is held tilted more than lift_tilt_deg for 200 ms
 *	                (picked up and carried).
 *	Short shocks (hump edges, wheel bumps) are filtered out by the low-pass
 *	filters and the hold times.
 *	Putting the robot DOWN looks like a lift at the moment it lands (the
 *	downward speed is stopped by an upward push). So after a clear downward
 *	movement the two "upward" rules are ignored for 1 s.
 *	Limit: a 20 cm lift spread over more than ~2 s never reaches the speed
 *	threshold; an accelerometer alone cannot see it.
 */

#ifndef B4_POSTURE_H
#define B4_POSTURE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    B4_LIFT_NONE = 0, B4_LIFT_LIFTED, B4_LIFT_OVERTURNED
} b4_lift_cause_t;

typedef struct
{
    uint16_t lift_accel_mg;      /* upward acceleration above 1 g */
    uint16_t lift_speed_mm_s;    /* upward speed (integrated acceleration) */
    uint8_t  lift_tilt_deg;      /* held tilt that means "picked up" */
    uint8_t  overturn_tilt_deg;  /* tilt that means "on its side / roof" */
} b4_posture_cfg_t;

typedef struct
{
    float   g0x, g0y, g0z;       /* calibrated gravity, unit vector */
    float   g0_mg;               /* calibrated gravity magnitude (~1000) */
    float   fx, fy, fz;          /* low-pass acceleration, for tilt */
    float   up_dev_mg;           /* low-pass vertical deviation from 1 g */
    float   up_speed_mm_s;       /* leaky-integrated vertical speed, + = up */
    float   tilt_deg;            /* angle from calibrated level, 0..180 */
    uint8_t up_count;
    uint8_t speed_count;
    uint8_t settle_count;        /* >0: just moved down, ignore "up" rules */
    uint8_t tilt_count;
    uint8_t flip_count;
} b4_posture_t;

#define B4_POSTURE_UP_HOLD      (5U)    /* 50 ms at 100 Hz */
#define B4_POSTURE_SPEED_HOLD   (10U)   /* 100 ms: a bump falls back sooner */
#define B4_POSTURE_SETTLE       (100U)  /* 1 s after moving down */
#define B4_POSTURE_TILT_HOLD    (20U)   /* 200 ms */
#define B4_POSTURE_FLIP_HOLD    (10U)   /* 100 ms */

/* gx/gy/gz: mean acceleration measured while calibrating (still, level). */
void b4_posture_init(b4_posture_t *p, float gx, float gy, float gz);

/* One sample (car frame, mg). Returns the cause once the condition has
   been held long enough, B4_LIFT_NONE otherwise. Does not latch: the
   caller latches. */
b4_lift_cause_t b4_posture_update(b4_posture_t *p, const b4_posture_cfg_t *cfg,
                                  float ax, float ay, float az);

#endif /* B4_POSTURE_H */
