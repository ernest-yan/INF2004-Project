/* PORT: <stddef.h>/<string.h> removed (they clash with the kernel's size_t).
 * This file is now fully self-contained: it needs only its own header, and a
 * plain 0 is used where NULL was (NULL is not available without those headers). */
#include "profile.h"

#include <stdbool.h>

/* INTEGRATION FIX: only the outer rays (>= 50 degrees off forward) count as
 * "sides".  The old 70/110 limits put the 70- and 110-degree rays in the side
 * sectors, and a box about 22 cm or wider at ~30 cm already covers those rays,
 * so both sides looked blocked (FAULT_NO_SAFE_SIDE).  The coarse 30/150 rays
 * are always there, so one valid ray per side is enough. */
#define subsystem5_LEFT_MIN_DEG   (140U)
#define subsystem5_RIGHT_MAX_DEG  (40U)
#define subsystem5_MIN_SIDE_RAYS  (1U)
#define subsystem5_PI_F           (3.14159265f)

/* CR1: a computed distance into a 12-bit field, saturating (never wraps). */
static uint32_t subsystem5_dist_field(uint32_t mm)
{
    return (mm > subsystem5_DIST_FIELD_MAX) ? subsystem5_DIST_FIELD_MAX : mm;
}

/* cos(deg) for 0..180 deg as sin(90 - deg), Taylor to x^9 (no libm here). */
float subsystem5_cosd(float deg)
{
    const float x = (90.0f - deg) * (subsystem5_PI_F / 180.0f);
    const float x2 = x * x;
    return x * (1.0f - x2 / 6.0f * (1.0f - x2 / 20.0f * (1.0f - x2 / 42.0f *
           (1.0f - x2 / 72.0f))));
}

/* Edge-based width from the fine sweep (see profile.h). Rays are in
 * increasing angle order: index 0 is the right-most. */
static void subsystem5_edge_width(const subsystem5_scan_samples_t *s,
                                  uint16_t obstacle_limit_mm,
                                  subsystem5_profile_t *p)
{
    uint8_t i, c = 0U, lo, hi;
    uint16_t best = UINT16_MAX;
    uint32_t face_mm;
    float a_lo, a_hi, x_lo, x_hi, w;

    for (i = 0U; i < subsystem5_FINE_COUNT; i++)
    {
        const subsystem5_sample_t *r = &s->fine[i];
        if ((r->valid != 0U) && (r->distance_mm <= obstacle_limit_mm) &&
            (r->distance_mm < best))
        {
            best = r->distance_mm;
            c = i;
        }
    }
    if (best == UINT16_MAX)
    {
        return;                 /* the fine sweep missed it: keep old width */
    }
    face_mm = (uint32_t)best + subsystem5_FACE_TOL_MM;
    lo = c;
    while ((lo > 0U) && (s->fine[lo - 1U].valid != 0U) &&
           (s->fine[lo - 1U].distance_mm <= face_mm))
    {
        lo--;
    }
    hi = c;
    while ((hi < (subsystem5_FINE_COUNT - 1U)) &&
           (s->fine[hi + 1U].valid != 0U) &&
           (s->fine[hi + 1U].distance_mm <= face_mm))
    {
        hi++;
    }
    p->right_edge_deg = s->fine[lo].angle_deg;
    p->left_edge_deg = s->fine[hi].angle_deg;
    p->right_edge_mm = s->fine[lo].distance_mm;
    p->left_edge_mm = s->fine[hi].distance_mm;
    p->width_capped = ((lo == 0U) || (hi == (subsystem5_FINE_COUNT - 1U))) ?
                      1U : 0U;

    /* Move each edge inward by the beam's half-width, never past each other. */
    a_lo = (float)s->fine[lo].angle_deg + (float)subsystem5_BEAM_EDGE_DEG;
    a_hi = (float)s->fine[hi].angle_deg - (float)subsystem5_BEAM_EDGE_DEG;
    if (a_lo > a_hi)
    {
        a_lo = ((float)s->fine[lo].angle_deg + (float)s->fine[hi].angle_deg) * 0.5f;
        a_hi = a_lo;
    }
    /* Sideways position, + = right of the sensor. */
    x_lo = (float)s->fine[lo].distance_mm * subsystem5_cosd(a_lo);
    x_hi = (float)s->fine[hi].distance_mm * subsystem5_cosd(a_hi);
    w = x_lo - x_hi;
    if (w < 0.0f) { w = 0.0f; }
    p->approximate_width_mm = subsystem5_dist_field((uint32_t)(w + 0.5f));
}

typedef struct
{
    uint16_t min_mm;
    uint8_t valid_count;
    uint8_t invalid_count;
} side_data_t;

static void subsystem5_consider_sample(const subsystem5_sample_t *s,
                               bool sides,
                               uint16_t obstacle_limit_mm,
                               subsystem5_profile_t *p,
                               uint16_t *first_angle,
                               uint16_t *last_angle,
                               uint8_t *hit_count,
                               side_data_t *left,
                               side_data_t *right)
{
    side_data_t *sector = 0;

    /* Side clearance comes from the coarse 30/150-degree rays only. Fine
       rays near the sides mostly hit the obstacle's own edges, which would
       make a wide obstacle look like a blocked side. */
    if (!sides)
    {
        /* forward / obstacle rays only */
    }
    else if (s->angle_deg >= subsystem5_LEFT_MIN_DEG)
    {
        sector = left;
    }
    else if (s->angle_deg <= subsystem5_RIGHT_MAX_DEG)
    {
        sector = right;
    }
    else
    {
        /* The forward sector is profiled separately. */
    }

    if ((s->valid == 0U) || (s->distance_mm == subsystem5_UNKNOWN_MM))
    {
        if (sector != 0)
        {
            sector->invalid_count++;
        }
        return;
    }

    if (sector != 0)
    {
        sector->valid_count++;
        if (s->distance_mm < sector->min_mm)
        {
            sector->min_mm = s->distance_mm;
        }
    }

    if (s->distance_mm <= obstacle_limit_mm)
    {
        p->obstacle_found = 1U;
        if ((*hit_count == 0U) || (s->angle_deg < *first_angle))
        {
            *first_angle = s->angle_deg;
        }
        if ((*hit_count == 0U) || (s->angle_deg > *last_angle))
        {
            *last_angle = s->angle_deg;
        }
        (*hit_count)++;

        if ((p->closest_mm == 0U) || (s->distance_mm < p->closest_mm))
        {
            p->closest_mm = s->distance_mm;
            p->closest_angle_deg = s->angle_deg;
        }
    }
}

subsystem5_profile_t subsystem5_profile_analyze(const subsystem5_scan_samples_t *samples,
                                uint16_t obstacle_limit_mm,
                                uint16_t min_clearance_mm)
{
    subsystem5_profile_t p;
    side_data_t left = { UINT16_MAX, 0U, 0U };
    side_data_t right = { UINT16_MAX, 0U, 0U };
    uint16_t first_angle = 0U;
    uint16_t last_angle = 0U;
    uint8_t hit_count = 0U;
    uint8_t i;
    uint8_t invalid_total = 0U;

    subsystem5_memset(&p, 0, (int)sizeof(p));

    if ((samples == 0) ||
        (samples->coarse_count != subsystem5_COARSE_COUNT) ||
        (samples->fine_count != subsystem5_FINE_COUNT))
    {
        return p;
    }

    for (i = 0U; i < subsystem5_COARSE_COUNT; i++)
    {
        subsystem5_consider_sample(&samples->coarse[i], true, obstacle_limit_mm,
                           &p, &first_angle, &last_angle, &hit_count,
                           &left, &right);
        invalid_total += ((samples->coarse[i].valid == 0U) ||
                          (samples->coarse[i].distance_mm == subsystem5_UNKNOWN_MM))
                         ? 1U : 0U;
    }
    for (i = 0U; i < subsystem5_FINE_COUNT; i++)
    {
        subsystem5_consider_sample(&samples->fine[i], false, obstacle_limit_mm,
                           &p, &first_angle, &last_angle, &hit_count,
                           &left, &right);
        invalid_total += ((samples->fine[i].valid == 0U) ||
                          (samples->fine[i].distance_mm == subsystem5_UNKNOWN_MM))
                         ? 1U : 0U;
    }

    if (hit_count > 0U)
    {
        p.span_deg = last_angle - first_angle;
        /* Fallback: small-angle chord/arc, width ~= r * theta(rad). */
        p.approximate_width_mm = subsystem5_dist_field(
            ((uint32_t)p.closest_mm * p.span_deg * 1745U) / 100000U);
        /* Preferred: edge-based width from the fine sweep. */
        subsystem5_edge_width(samples, obstacle_limit_mm, &p);
    }

    /* Unknown rays cannot be interpreted as open space. */
    if ((left.valid_count >= subsystem5_MIN_SIDE_RAYS) &&
        (left.invalid_count == 0U))
    {
        p.left_clearance_valid = 1U;
        p.left_clearance_mm = left.min_mm;
    }
    if ((right.valid_count >= subsystem5_MIN_SIDE_RAYS) &&
        (right.invalid_count == 0U))
    {
        p.right_clearance_valid = 1U;
        p.right_clearance_mm = right.min_mm;
    }

    /* Any invalid measurement makes the profile incomplete. One rescan allowed. */
    p.profile_valid = (invalid_total == 0U) ? 1U : 0U;

    if ((p.obstacle_found != 0U) && (p.profile_valid != 0U))
    {
        const bool left_ok = (p.left_clearance_valid != 0U) &&
                             (p.left_clearance_mm >= min_clearance_mm);
        const bool right_ok = (p.right_clearance_valid != 0U) &&
                              (p.right_clearance_mm >= min_clearance_mm);

        if (left_ok && right_ok)
        {
            p.candidate_side =
                (p.left_clearance_mm >= p.right_clearance_mm) ?
                    subsystem5_SIDE_LEFT : subsystem5_SIDE_RIGHT;
        }
        else if (left_ok)
        {
            p.candidate_side = subsystem5_SIDE_LEFT;
        }
        else if (right_ok)
        {
            p.candidate_side = subsystem5_SIDE_RIGHT;
        }
    }

    return p;
}