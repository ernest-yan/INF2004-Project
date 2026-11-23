/*
 * motion_cal.h
 * Subsystem 2: the robot's measured wheel and encoder calibration.
 *
 * This is the ONLY place these numbers live. S2 builds its b2_config_t from
 * them (tkernel_adapter.c) and S5 derives its bypass tick scales from them
 * (subsystem5/ultrasonic.h), so a re-calibration reaches both.
 *
 * How to measure: run the calibration build (make TEST=1 S2_CAL=1) and
 * follow testing/subsystem2/CALIBRATION.md. It prints the new values.
 */
#ifndef B2_MOTION_CAL_H
#define B2_MOTION_CAL_H
#include <stdint.h>

/* Encoder edges per wheel revolution, per wheel. 2026-11-08 hand-turn
 * measurement (10 turns) gave L 962 / R 896; the two discs should be
 * identical, so the 7 % gap is unconfirmed and both wheels still use the
 * mean (930). Re-measure with S2-C01 before using per-wheel values. */
#ifndef B2_CAL_TICKS_PER_REV_LEFT
#define B2_CAL_TICKS_PER_REV_LEFT       (930U)
#endif
#ifndef B2_CAL_TICKS_PER_REV_RIGHT
#define B2_CAL_TICKS_PER_REV_RIGHT      (930U)
#endif

/* Circumference: PROVISIONAL until S2-C02 is run on the floor. */
#ifndef B2_CAL_WHEEL_CIRCUMFERENCE_MM
#define B2_CAL_WHEEL_CIRCUMFERENCE_MM   (210U)
#endif
/* Track: 110 mm, measured with a ruler 2026-11-20 (centre of one drive
 * tyre to the centre of the other); was a provisional 120. S2-C03/C04 (turn
 * 360 deg) can refine it: skid on the floor can make the effective track a
 * little wider than the measured one. */
#ifndef B2_CAL_WHEEL_TRACK_MM
#define B2_CAL_WHEEL_TRACK_MM           (110U)
#endif

/* Mean of the two wheels, for code that only counts one "robot" tick
 * (S5's bypass legs). Rounded to nearest. */
#define B2_CAL_TICKS_PER_REV_MEAN                                        \
    ((B2_CAL_TICKS_PER_REV_LEFT + B2_CAL_TICKS_PER_REV_RIGHT + 1U) / 2U)

/* Encoder ticks per metre of travel, rounded to nearest. */
#define B2_CAL_TICKS_PER_M                                               \
    (((B2_CAL_TICKS_PER_REV_MEAN * 1000U) +                              \
      (B2_CAL_WHEEL_CIRCUMFERENCE_MM / 2U)) / B2_CAL_WHEEL_CIRCUMFERENCE_MM)

/* Ticks per wheel per 0.1 degree of an on-the-spot turn, rounded to
 * nearest: ticks/rev * (pi * track / 360) / circumference * 10, with
 * pi = 31416 / 10000. 64-bit: the product passes 2^32 above ~145 mm track. */
#define B2_CAL_TURN_TICKS_PER_DEG_X10                                    \
    ((uint32_t)((((uint64_t)B2_CAL_TICKS_PER_REV_MEAN *                  \
                  B2_CAL_WHEEL_TRACK_MM * 31416ULL) +                    \
                 (((uint64_t)B2_CAL_WHEEL_CIRCUMFERENCE_MM * 360000ULL) / \
                  2ULL)) /                                               \
                ((uint64_t)B2_CAL_WHEEL_CIRCUMFERENCE_MM * 360000ULL)))

#endif /* B2_MOTION_CAL_H */
