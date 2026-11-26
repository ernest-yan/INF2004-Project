/*
 *      motion_class.h
 *      Subsystem 4 -- turn rate and motion classification.
 *
 *      Pure maths, no hardware (tested in subsystem4_test.c, S4-U09/U10).
 *      The GY-511 has no gyroscope, so the turn (yaw) rate comes from the
 *      difference between S2's left and right wheel speeds:
 *          yaw rate = (right - left) / wheel track    [rad/s]
 *      Positive = turning left (anticlockwise seen from above).
 *      It is wrong while a wheel slips or the robot is lifted.
 */

#ifndef B4_MOTION_CLASS_H
#define B4_MOTION_CLASS_H

#include <stdbool.h>
#include <stdint.h>
#include "posture.h"
#include "terrain.h"

typedef enum
{
    B4_MOTION_CALIBRATING = 0, B4_MOTION_STATIONARY, B4_MOTION_MOVING,
    B4_MOTION_TURNING, B4_MOTION_ON_HUMP, B4_MOTION_LIFTED,
    B4_MOTION_OVERTURNED
} b4_motion_t;                          /* 7 values -> 3 bits on the wire */

#define B4_STILL_MM_S        (20)       /* both wheels slower = stationary */
#define B4_TURNING_DPS       (45.0f)    /* PROVISIONAL: above line-follow
                                           corrections, below pivot turns */

float b4_yaw_rate_dps(int32_t left_mm_s, int32_t right_mm_s, uint16_t track_mm);

b4_motion_t b4_classify_motion(bool calibrated, b4_lift_cause_t lift,
                               b4_terrain_phase_t phase,
                               int32_t left_mm_s, int32_t right_mm_s,
                               float yaw_dps);

#endif /* B4_MOTION_CLASS_H */
