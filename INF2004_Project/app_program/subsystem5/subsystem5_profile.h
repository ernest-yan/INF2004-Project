#ifndef SUBSYSTEM5_PROFILE_H
#define SUBSYSTEM5_PROFILE_H

#include <stdint.h>

/* PORT: <string.h> is not used (the kernel's tk/syslib.h already defines
 * size_t, so a second definition would not compile).  This tiny helper
 * replaces memset().  */
static inline void subsystem5_memset(void *dest, int value, int count)
{
    unsigned char *p = (unsigned char *)dest;
    while (count-- > 0)
    {
        *p++ = (unsigned char)value;
    }
}

/* Five coarse rays (30, 60, 90, 120, 150 degrees), then five fine rays. */
#define subsystem5_COARSE_COUNT (5U)
#define subsystem5_FINE_COUNT   (5U)
#define subsystem5_UNKNOWN_MM   UINT16_MAX

typedef enum
{
    subsystem5_SIDE_NONE = 0,
    subsystem5_SIDE_LEFT,
    subsystem5_SIDE_RIGHT
} subsystem5_side_t;

typedef struct
{
    uint16_t angle_deg;
    uint16_t distance_mm;
    uint8_t valid; /* Zero for a timeout or unusable echo. */
} subsystem5_sample_t;

typedef struct
{
    subsystem5_sample_t coarse[subsystem5_COARSE_COUNT];
    subsystem5_sample_t fine[subsystem5_FINE_COUNT];
    uint8_t coarse_count;
    uint8_t fine_count;
} subsystem5_scan_samples_t;

typedef struct
{
    uint8_t obstacle_found;
    uint8_t profile_valid;
    uint16_t closest_mm;
    uint16_t closest_angle_deg;
    uint16_t span_deg;
    uint16_t approximate_width_mm;
    uint16_t left_clearance_mm;
    uint16_t right_clearance_mm;
    uint8_t left_clearance_valid;
    uint8_t right_clearance_valid;
    subsystem5_side_t candidate_side;
} subsystem5_profile_t;

/* Angles increase from right to left; 90 degrees points forward.
 * A null/incomplete scan returns an invalid, zero-initialized profile.
 * Unknown/invalid rays cannot establish clearance. SIDE_NONE means no
 * bypass recommendation; the caller remains responsible for motion. */
subsystem5_profile_t subsystem5_profile_analyze(
    const subsystem5_scan_samples_t *samples,
    uint16_t obstacle_limit_mm,
    uint16_t min_clearance_mm);

#endif /* SUBSYSTEM5_PROFILE_H */