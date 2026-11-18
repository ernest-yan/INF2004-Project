/* CR1 S5 equivalence test: plain (unpacked) scan and profile, shared by the
 * pre-CR1 and the CR1 builds of profile.c. See run_host_tests.sh. */
#ifndef S5_COMMON_H
#define S5_COMMON_H
#include <stdint.h>

typedef struct
{
    uint16_t angle_deg;
    uint16_t distance_mm;
    uint8_t valid;
    uint8_t unknown;            /* distance = that build's UNKNOWN sentinel */
} plain_sample_t;

typedef struct
{
    plain_sample_t coarse[5];
    plain_sample_t fine[15];
    uint8_t coarse_count;
    uint8_t fine_count;
} plain_scan_t;

typedef struct
{
    uint32_t obstacle_found, profile_valid, closest_mm, closest_angle_deg;
    uint32_t span_deg, approximate_width_mm, left_clearance_mm;
    uint32_t right_clearance_mm, left_clearance_valid, right_clearance_valid;
    uint32_t candidate_side, right_edge_deg, left_edge_deg, right_edge_mm;
    uint32_t left_edge_mm, width_capped;
} plain_profile_t;

void old_analyze(const plain_scan_t *in, uint16_t limit_mm, uint16_t clear_mm,
                 plain_profile_t *out, uint32_t *sizes);
void new_analyze(const plain_scan_t *in, uint16_t limit_mm, uint16_t clear_mm,
                 plain_profile_t *out, uint32_t *sizes);
#endif
