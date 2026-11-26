/*
 *      motion_class.c
 *      Subsystem 4 -- turn rate and motion classification (see motion_class.h).
 */

#include "motion_class.h"
#include "mathf.h"

#define B4_RAD_TO_DEG   (57.2957795f)

float b4_yaw_rate_dps(int32_t left_mm_s, int32_t right_mm_s, uint16_t track_mm)
{
    if (track_mm == 0U) { return 0.0f; }
    return ((float)(right_mm_s - left_mm_s) / (float)track_mm) * B4_RAD_TO_DEG;
}

b4_motion_t b4_classify_motion(bool calibrated, b4_lift_cause_t lift,
                               b4_terrain_phase_t phase,
                               int32_t left_mm_s, int32_t right_mm_s,
                               float yaw_dps)
{
    const bool still = (left_mm_s > -B4_STILL_MM_S) &&
                       (left_mm_s < B4_STILL_MM_S) &&
                       (right_mm_s > -B4_STILL_MM_S) &&
                       (right_mm_s < B4_STILL_MM_S);

    /* Most important first. */
    if (!calibrated)                       { return B4_MOTION_CALIBRATING; }
    if (lift == B4_LIFT_OVERTURNED)        { return B4_MOTION_OVERTURNED; }
    if (lift == B4_LIFT_LIFTED)            { return B4_MOTION_LIFTED; }
    if (phase != B4_TERRAIN_LEVEL)         { return B4_MOTION_ON_HUMP; }
    if (still)                             { return B4_MOTION_STATIONARY; }
    if (b4_fabsf(yaw_dps) >= B4_TURNING_DPS) { return B4_MOTION_TURNING; }
    return B4_MOTION_MOVING;
}
