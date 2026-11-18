/**
 * CR1 host test for subsystem 5: the pre-CR1 and the CR1 profile.c analyse
 * the SAME random scans and must produce the SAME obstacle profile, field by
 * field (location, width, closest point, clearances, chosen side). Run with
 * run_host_tests.sh, which builds both versions.
 */
#include "s5_common.h"
#include <stdio.h>

static uint32_t g_rng = 12345U;
static uint32_t rnd(uint32_t n)
{
    g_rng = (g_rng * 1103515245U) + 12345U;
    return (g_rng >> 8) % n;
}

/* A ray: mostly realistic distances, some far (4000), some no echo. */
static void ray(plain_sample_t *s, uint16_t angle)
{
    const uint32_t kind = rnd(100U);
    s->angle_deg = angle;
    s->unknown = 0U;
    s->valid = 1U;
    if (kind < 4U)       { s->valid = 0U; s->unknown = 1U; s->distance_mm = 0U; }
    else if (kind < 20U) { s->distance_mm = 4000U; }
    else if (kind < 60U) { s->distance_mm = (uint16_t)(150U + rnd(500U)); }
    else                 { s->distance_mm = (uint16_t)(20U + rnd(3980U)); }
}

int main(void)
{
    static const uint16_t coarse[5] = {30U, 60U, 90U, 120U, 150U};
    uint32_t sizes_old[3], sizes_new[3];
    uint32_t mismatches = 0U, obstacles = 0U, sides = 0U;
    const uint32_t runs = 200000U;
    for (uint32_t r = 0U; r < runs; r++)
    {
        plain_scan_t scan;
        plain_profile_t a, b;
        const uint16_t centre = coarse[1U + rnd(3U)];
        for (uint32_t i = 0U; i < 5U; i++) { ray(&scan.coarse[i], coarse[i]); }
        for (uint32_t i = 0U; i < 15U; i++)
        {
            ray(&scan.fine[i], (uint16_t)(centre - 35U + (5U * i)));
        }
        scan.coarse_count = (rnd(50U) == 0U) ? 4U : 5U;   /* some incomplete */
        scan.fine_count = 15U;
        old_analyze(&scan, 500U, 400U, &a, sizes_old);
        new_analyze(&scan, 500U, 400U, &b, sizes_new);
        const uint32_t *pa = (const uint32_t *)&a;
        const uint32_t *pb = (const uint32_t *)&b;
        for (uint32_t f = 0U; f < sizeof(a) / sizeof(uint32_t); f++)
        {
            if (pa[f] != pb[f])
            {
                if (mismatches < 10U)
                {
                    printf("MISMATCH run %u field %u: old %u new %u\n",
                           (unsigned)r, (unsigned)f, (unsigned)pa[f],
                           (unsigned)pb[f]);
                }
                mismatches++;
            }
        }
        obstacles += a.obstacle_found;
        sides += (a.candidate_side != 0U) ? 1U : 0U;
    }
    printf("S5 profile equivalence: %u random scans (%u with an obstacle, "
           "%u with a bypass side), %u field mismatches\n",
           (unsigned)runs, (unsigned)obstacles, (unsigned)sides,
           (unsigned)mismatches);
    printf("  sizeof sample %u -> %u, scan %u -> %u, profile %u -> %u\n",
           (unsigned)sizes_old[0], (unsigned)sizes_new[0],
           (unsigned)sizes_old[1], (unsigned)sizes_new[1],
           (unsigned)sizes_old[2], (unsigned)sizes_new[2]);
    return (mismatches == 0U) ? 0 : 1;
}
