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

typedef struct
{
    uint16_t min_mm;
    uint8_t valid_count;
    uint8_t invalid_count;
} side_data_t;

static void subsystem5_consider_sample(const subsystem5_sample_t *s,
                               uint16_t obstacle_limit_mm,
                               subsystem5_profile_t *p,
                               uint16_t *first_angle,
                               uint16_t *last_angle,
                               uint8_t *hit_count,
                               side_data_t *left,
                               side_data_t *right)
{
    side_data_t *sector = 0;

    if (s->angle_deg >= subsystem5_LEFT_MIN_DEG)
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
        subsystem5_consider_sample(&samples->coarse[i], obstacle_limit_mm,
                           &p, &first_angle, &last_angle, &hit_count,
                           &left, &right);
        invalid_total += ((samples->coarse[i].valid == 0U) ||
                          (samples->coarse[i].distance_mm == subsystem5_UNKNOWN_MM))
                         ? 1U : 0U;
    }
    for (i = 0U; i < subsystem5_FINE_COUNT; i++)
    {
        subsystem5_consider_sample(&samples->fine[i], obstacle_limit_mm,
                           &p, &first_angle, &last_angle, &hit_count,
                           &left, &right);
        invalid_total += ((samples->fine[i].valid == 0U) ||
                          (samples->fine[i].distance_mm == subsystem5_UNKNOWN_MM))
                         ? 1U : 0U;
    }

    if (hit_count > 0U)
    {
        p.span_deg = last_angle - first_angle;
        /* Small-angle chord/arc approximation: width ~= r * theta(rad). */
        p.approximate_width_mm =
            (uint16_t)(((uint32_t)p.closest_mm * p.span_deg * 1745U)
                         / 100000U);
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