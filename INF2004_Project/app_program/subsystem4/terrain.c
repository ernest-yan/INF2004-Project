/*
 *      terrain.c
 *      Subsystem 4 -- hump detection and height, peak-tilt method (terrain.h).
 */
#include "terrain.h"
#include "mathf.h"   /* PORT: replaces <math.h> (no libm linked) */
#include <string.h>

/* All units mm. Demo hump (23 Nov) is 50 mm high: it tilts the car
   ~20-30 deg, far above the start threshold. */
#define B4_HUMP_START_DEG      (2.0f)   /* tilt that may be a hump */
#define B4_HUMP_START_SAMPLES  (8U)     /* 80 ms tilted, while moving */
#define B4_HUMP_PHASE_MM       (30U)    /* a solid half: tilted over 30 mm of
                                           driving. Pulling away or braking
                                           tilts the car over ~10 mm. */
#define B4_HUMP_CLEAR_MM       (80U)    /* travel with no clear tilt = done */
#define B4_HUMP_STILL_SAMPLES  (100U)   /* or stopped 1 s after both phases */
#define B4_HUMP_MAX_PATH_MM    (1000U)  /* longer = a ramp or noise. Was 600:
                                           a long 50 mm hump + wheelbase +
                                           80 mm clear could pass 600. */
#define B4_MIN_HUMP_MM         (3.0f)
#define B4_MAX_HUMP_MM         (150.0f)
#define B4_PITCH_ALPHA         (0.1f)   /* +100 ms smoothing at 100 Hz */
#define B4_DEG_TO_RAD          (0.0174532925f)

/* CR1: heights are 8-bit (0..255 mm), saturating. */
static uint8_t b4_u8_round(float v)
{
    if (v <= 0.0f) { return 0U; }
    if (v >= 255.0f) { return 255U; }
    return (uint8_t)(v + 0.5f);
}

/* CR1: 8-bit saturating sample counter. */
static uint8_t b4_inc(uint8_t n)
{
    return (n < 255U) ? (uint8_t)(n + 1U) : n;
}

static float b4_height_mm(const b4_terrain_t *m, uint16_t wheelbase_mm)
{
    /* Use each half only if it is solid; average when both are. */
    const bool up_ok = (m->up_mm >= B4_HUMP_PHASE_MM);
    const bool dn_ok = (m->down_mm >= B4_HUMP_PHASE_MM);
    const float up = b4_sinf(m->max_pitch_deg * B4_DEG_TO_RAD);
    const float dn = b4_sinf(-m->min_pitch_deg * B4_DEG_TO_RAD);
    float s = 0.0f;

    if (up_ok && dn_ok) { s = (up + dn) * 0.5f; }
    else if (up_ok)     { s = up; }
    else if (dn_ok)     { s = dn; }
    return (float)wheelbase_mm * s;
}

/* Back to level, keeping the results of completed humps. */
static void b4_terrain_rearm(b4_terrain_t *m)
{
    m->phase = B4_TERRAIN_LEVEL;
    m->max_pitch_deg = 0.0f;
    m->min_pitch_deg = 0.0f;
    m->path_mm = 0U;
    m->max_at_mm = 0U;
    m->min_at_mm = 0U;
    m->up_mm = 0U;
    m->down_mm = 0U;
    m->start_samples = 0U;
    m->still_samples = 0U;
    m->clear_mm = 0U;
    m->peak_mm = 0U;
}

void b4_terrain_init(b4_terrain_t *model)
{
    if (model != NULL) { memset(model, 0, sizeof(*model)); }
}

bool b4_terrain_update(b4_terrain_t *model, float pitch_deg,
                       uint32_t step_mm, uint16_t wheelbase_mm)
{
    b4_terrain_t *m = model;
    float p;
    bool moving;

    if ((m == NULL) || (wheelbase_mm == 0U)) { return false; }
    /* A single huge encoder delta indicates slip/communication fault. */
    if (step_mm > 100U) { step_mm = 0U; }
    moving = (step_mm > 0U);

    m->pitch_deg += B4_PITCH_ALPHA * (pitch_deg - m->pitch_deg);
    p = m->pitch_deg;

    if (m->phase == B4_TERRAIN_LEVEL)
    {
        /* Only while driving: a car stopped at an angle is not a hump. */
        if (moving && (b4_fabsf(p) >= B4_HUMP_START_DEG))
        {
            m->start_samples = b4_inc(m->start_samples);
        }
        else if (b4_fabsf(p) < B4_HUMP_START_DEG)
        {
            m->start_samples = 0U;
        }
        if (m->start_samples >= B4_HUMP_START_SAMPLES)
        {
            b4_terrain_rearm(m);
            m->phase = (p > 0.0f) ? B4_TERRAIN_CLIMB : B4_TERRAIN_DESCEND;
        }
        else
        {
            return false;
        }
    }

    m->path_mm = (uint16_t)(m->path_mm + step_mm);  /* step <= 100 */
    if (m->path_mm > B4_HUMP_MAX_PATH_MM)
    {
        /* Too long for a hump: a ramp, a slope, or noise. Never report. */
        b4_terrain_rearm(m);
        return false;
    }

    /* Only tilt seen WHILE DRIVING belongs to the hump. A stopped car
       being picked up or repositioned must not change the result. */
    if (moving)
    {
        if (p > m->max_pitch_deg) { m->max_pitch_deg = p; m->max_at_mm = m->path_mm; }
        if (p < m->min_pitch_deg) { m->min_pitch_deg = p; m->min_at_mm = m->path_mm; }
        if (p >= B4_HUMP_START_DEG)
        {
            m->up_mm = (uint16_t)(m->up_mm + step_mm);
            m->phase = B4_TERRAIN_CLIMB;
        }
        else if (p <= -B4_HUMP_START_DEG)
        {
            m->down_mm = (uint16_t)(m->down_mm + step_mm);
            m->phase = B4_TERRAIN_DESCEND;
        }
    }
    m->peak_mm = b4_u8_round(b4_height_mm(m, wheelbase_mm));

    /* Finished after enough travel with no clear tilt (small wobble does
       not count), or when the car has stopped after both phases. */
    if (moving && (b4_fabsf(p) >= B4_HUMP_START_DEG))
    {
        m->clear_mm = 0U;
    }
    else
    {
        m->clear_mm = (uint16_t)(m->clear_mm + step_mm);
    }
    m->still_samples = moving ? 0U : b4_inc(m->still_samples);
    {
        const bool both = (m->up_mm >= B4_HUMP_PHASE_MM) &&
                          (m->down_mm >= B4_HUMP_PHASE_MM);
        /* With one phase seen so far, the level stretch between the two
           peaks can be up to a wheelbase long: wait past it. */
        const uint32_t clear_needed = both ? B4_HUMP_CLEAR_MM :
                                      ((uint32_t)wheelbase_mm + 40U);
        if ((m->clear_mm < clear_needed) &&
            (m->still_samples < B4_HUMP_STILL_SAMPLES))
        {
            return false;
        }
        /* Stopped for 1 s: measure what was seen while driving and close
           the hump, so nothing carries into the next run. */
    }

    {
        const float h = b4_height_mm(m, wheelbase_mm);
        const bool any = (m->up_mm >= B4_HUMP_PHASE_MM) ||
                         (m->down_mm >= B4_HUMP_PHASE_MM);
        const bool complete = any && (h >= B4_MIN_HUMP_MM) &&
                              (h <= B4_MAX_HUMP_MM);
        if (complete)
        {
            const uint8_t peak = b4_u8_round(h);
            m->last_peak_mm = peak;
            if (peak > m->max_peak_mm) { m->max_peak_mm = peak; }
            m->last_max_cdeg = (int16_t)(m->max_pitch_deg * 100.0f);
            m->last_min_cdeg = (int16_t)(m->min_pitch_deg * 100.0f);
            m->last_gap_mm = (uint16_t)((m->max_at_mm > m->min_at_mm) ?
                                        (m->max_at_mm - m->min_at_mm) :
                                        (m->min_at_mm - m->max_at_mm));
        }
        b4_terrain_rearm(m);
        return complete;
    }
}
