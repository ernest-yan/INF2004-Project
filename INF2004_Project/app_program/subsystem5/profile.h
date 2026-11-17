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

/* Five coarse rays (30, 60, 90, 120, 150 degrees), then a fine sweep of 15
 * rays 5 degrees apart (70 degrees) centred on the closest coarse ray. */
#define subsystem5_COARSE_COUNT (5U)
#define subsystem5_FINE_COUNT   (15U)
#define subsystem5_FINE_STEP_DEG (5U)

/* Width from the obstacle's EDGES (briefing: W = f(dL, dC, dR, theta)).
 * A fine ray belongs to the obstacle's front face if it is within
 * FACE_TOL_MM of the closest reading. The left- and right-most such rays
 * are the edges; each edge's sideways position is d * cos(angle), and the
 * width is the distance between them. The HC-SR04 beam is several degrees
 * wide, so it still "sees" an edge after the ray has passed it: each edge
 * is moved inward by BEAM_EDGE_DEG. PROVISIONAL: calibrate with two boxes
 * of known width (raise it if widths read high, lower it if low). */
#define subsystem5_FACE_TOL_MM   (100U)
#define subsystem5_BEAM_EDGE_DEG (5U)
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
    uint16_t right_edge_deg;     /* fine-scan edge rays (before correction) */
    uint16_t left_edge_deg;
    uint16_t right_edge_mm;
    uint16_t left_edge_mm;
    uint8_t width_capped;        /* an edge was at the end of the sweep:
                                    the width is a lower bound */
} subsystem5_profile_t;

/* Angles increase from right to left; 90 degrees points forward.
 * A null/incomplete scan returns an invalid, zero-initialized profile.
 * Unknown/invalid rays cannot establish clearance. SIDE_NONE means no
 * bypass recommendation; the caller remains responsible for motion. */
/* cos() of an angle in degrees, 0..180 (no libm in this firmware). */
float subsystem5_cosd(float deg);

subsystem5_profile_t subsystem5_profile_analyze(
    const subsystem5_scan_samples_t *samples,
    uint16_t obstacle_limit_mm,
    uint16_t min_clearance_mm);

#endif /* SUBSYSTEM5_PROFILE_H */