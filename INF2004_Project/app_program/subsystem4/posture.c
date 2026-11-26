/*
 *      posture.c
 *      Subsystem 4 -- lifted / overturned detection (see posture.h).
 */

#include <stddef.h>  /* NULL; safe here: no kernel headers in this file */
#include "posture.h"
#include "mathf.h"   /* PORT: replaces <math.h> (no libm linked) */

#define B4_RAD_TO_DEG       (57.2957795f)
#define B4_TILT_ALPHA       (0.1f)  /* ~100 ms time constant at 100 Hz */
#define B4_UP_ALPHA         (0.3f)  /* ~30 ms: rejects single-sample shocks */
#define B4_SAMPLE_S         (0.01f) /* b4_poll period */
#define B4_MG_TO_MM_S2      (9.80665f)
#define B4_SPEED_LEAK       (0.99f) /* 1 s time constant: forgets sensor bias */

static uint8_t b4_count(uint8_t n, bool cond)
{
    if (!cond) { return 0U; }
    return (n < 255U) ? (uint8_t)(n + 1U) : n;
}

void b4_posture_init(b4_posture_t *p, float gx, float gy, float gz)
{
    float mag;

    if (p == NULL) { return; }
    mag = b4_sqrtf((gx * gx) + (gy * gy) + (gz * gz));
    if (mag < 1.0f)
    {
        /* Impossible at rest; fall back to "Z is up" so we never divide
           by zero. imu.c rejects such a calibration anyway. */
        gx = 0.0f; gy = 0.0f; gz = 1000.0f; mag = 1000.0f;
    }
    p->g0x = gx / mag;
    p->g0y = gy / mag;
    p->g0z = gz / mag;
    p->g0_mg = mag;
    p->fx = gx;
    p->fy = gy;
    p->fz = gz;
    p->up_dev_mg = 0.0f;
    p->up_speed_mm_s = 0.0f;
    p->tilt_deg = 0.0f;
    p->up_count = 0U;
    p->speed_count = 0U;
    p->settle_count = 0U;
    p->tilt_count = 0U;
    p->flip_count = 0U;
}

b4_lift_cause_t b4_posture_update(b4_posture_t *p, const b4_posture_cfg_t *cfg,
                                  float ax, float ay, float az)
{
    float up_mg, dot, cx, cy, cz, cross, fmag, vert_mg;

    if ((p == NULL) || (cfg == NULL)) { return B4_LIFT_NONE; }

    /* Upward speed. Vertical acceleration is taken along the CURRENT
       gravity direction (the low-pass vector from the previous samples),
       so a steady tilt reads 0, and vibration, being zero on average,
       cancels in the integral. The leak removes slow sensor drift. */
    fmag = b4_sqrtf((p->fx * p->fx) + (p->fy * p->fy) + (p->fz * p->fz));
    if (fmag > 1.0f)
    {
        vert_mg = (((ax * p->fx) + (ay * p->fy) + (az * p->fz)) / fmag)
                  - p->g0_mg;
        p->up_speed_mm_s = (p->up_speed_mm_s * B4_SPEED_LEAK) +
                           (vert_mg * B4_MG_TO_MM_S2 * B4_SAMPLE_S);
    }

    /* Vertical acceleration: component along calibrated "up", minus 1 g.
       Positive = being accelerated upwards (lifted). */
    up_mg = (ax * p->g0x) + (ay * p->g0y) + (az * p->g0z);
    p->up_dev_mg += B4_UP_ALPHA * ((up_mg - p->g0_mg) - p->up_dev_mg);

    /* Tilt: angle between the low-pass acceleration and calibrated up.
       atan2(|f x g0|, f . g0) is accurate at every angle (unlike acos). */
    p->fx += B4_TILT_ALPHA * (ax - p->fx);
    p->fy += B4_TILT_ALPHA * (ay - p->fy);
    p->fz += B4_TILT_ALPHA * (az - p->fz);
    dot = (p->fx * p->g0x) + (p->fy * p->g0y) + (p->fz * p->g0z);
    cx = (p->fy * p->g0z) - (p->fz * p->g0y);
    cy = (p->fz * p->g0x) - (p->fx * p->g0z);
    cz = (p->fx * p->g0y) - (p->fy * p->g0x);
    cross = b4_sqrtf((cx * cx) + (cy * cy) + (cz * cz));
    p->tilt_deg = b4_atan2f(cross, dot) * B4_RAD_TO_DEG;

    p->flip_count = b4_count(p->flip_count,
                             p->tilt_deg >= (float)cfg->overturn_tilt_deg);
    p->tilt_count = b4_count(p->tilt_count,
                             p->tilt_deg >= (float)cfg->lift_tilt_deg);
    p->up_count = b4_count(p->up_count,
                           p->up_dev_mg >= (float)cfg->lift_accel_mg);
    p->speed_count = b4_count(p->speed_count,
                         p->up_speed_mm_s >= (float)cfg->lift_speed_mm_s);

    /* Being put down: moving down at more than half the lift speed. The
       landing then reads as an upward push, so ignore the "up" rules
       until it has settled. */
    if (p->up_speed_mm_s <= -(0.5f * (float)cfg->lift_speed_mm_s))
    {
        p->settle_count = B4_POSTURE_SETTLE;
    }
    else if (p->settle_count > 0U)
    {
        p->settle_count--;
    }
    if (p->settle_count > 0U)
    {
        p->speed_count = 0U;
        p->up_count = 0U;
    }

    /* Most specific first. */
    if (p->flip_count >= B4_POSTURE_FLIP_HOLD) { return B4_LIFT_OVERTURNED; }
    if ((p->speed_count >= B4_POSTURE_SPEED_HOLD) ||
        (p->up_count >= B4_POSTURE_UP_HOLD) ||
        (p->tilt_count >= B4_POSTURE_TILT_HOLD))
    {
        return B4_LIFT_LIFTED;
    }
    return B4_LIFT_NONE;
}
