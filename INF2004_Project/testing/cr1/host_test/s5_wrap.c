/* Compiled twice by run_host_tests.sh: once against the pre-CR1 profile.c,
 * once against the CR1 one, with WRAP_NAME = old_analyze / new_analyze. */
#include "s5_common.h"
#include "profile.c"

static void fill(subsystem5_sample_t *d, const plain_sample_t *s)
{
    d->angle_deg = s->angle_deg;
    d->distance_mm = s->unknown ? subsystem5_UNKNOWN_MM : s->distance_mm;
    d->valid = s->valid;
}

void WRAP_NAME(const plain_scan_t *in, uint16_t limit_mm, uint16_t clear_mm,
               plain_profile_t *out, uint32_t *sizes)
{
    subsystem5_scan_samples_t s;
    subsystem5_profile_t p;
    uint32_t i;
    subsystem5_memset(&s, 0, (int)sizeof(s));
    for (i = 0U; i < 5U; i++) { fill(&s.coarse[i], &in->coarse[i]); }
    for (i = 0U; i < 15U; i++) { fill(&s.fine[i], &in->fine[i]); }
    s.coarse_count = in->coarse_count;
    s.fine_count = in->fine_count;
    p = subsystem5_profile_analyze(&s, limit_mm, clear_mm);
    out->obstacle_found = p.obstacle_found;
    out->profile_valid = p.profile_valid;
    out->closest_mm = p.closest_mm;
    out->closest_angle_deg = p.closest_angle_deg;
    out->span_deg = p.span_deg;
    out->approximate_width_mm = p.approximate_width_mm;
    out->left_clearance_mm = p.left_clearance_mm;
    out->right_clearance_mm = p.right_clearance_mm;
    out->left_clearance_valid = p.left_clearance_valid;
    out->right_clearance_valid = p.right_clearance_valid;
    out->candidate_side = p.candidate_side;
    out->right_edge_deg = p.right_edge_deg;
    out->left_edge_deg = p.left_edge_deg;
    out->right_edge_mm = p.right_edge_mm;
    out->left_edge_mm = p.left_edge_mm;
    out->width_capped = p.width_capped;
    sizes[0] = (uint32_t)sizeof(subsystem5_sample_t);
    sizes[1] = (uint32_t)sizeof(subsystem5_scan_samples_t);
    sizes[2] = (uint32_t)sizeof(subsystem5_profile_t);
}
