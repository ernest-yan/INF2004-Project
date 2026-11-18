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
/* CR1: distances are 12-bit fields (sensor range 0..4000 mm, see FAR_MM in
 * ultrasonic.c), so "unknown" is the all-ones 12-bit value. */
#define subsystem5_DIST_FIELD_MAX (4095U)
#define subsystem5_UNKNOWN_MM   subsystem5_DIST_FIELD_MAX

typedef enum
{
    subsystem5_SIDE_NONE = 0,
    subsystem5_SIDE_LEFT,
    subsystem5_SIDE_RIGHT
} subsystem5_side_t;

/* CR1: 6 -> 4 bytes. */
typedef struct
{
    uint32_t angle_deg : 8;     /* 0..180 */
    uint32_t distance_mm : 12;  /* 0..4000, subsystem5_UNKNOWN_MM = no echo */
    uint32_t valid : 1;         /* Zero for a timeout or unusable echo. */
    uint32_t : 11;
} subsystem5_sample_t;

typedef struct
{
    subsystem5_sample_t coarse[subsystem5_COARSE_COUNT];
    subsystem5_sample_t fine[subsystem5_FINE_COUNT];
    uint8_t coarse_count;
    uint8_t fine_count;
} subsystem5_scan_samples_t;

/* CR1: 28 -> 16 bytes. Distances 12 bits (0..4095 mm), angles 8 bits
 * (0..180 deg), flags 1 bit, side 2 bits. All unsigned (Barr C). */
typedef struct
{
    uint32_t closest_mm : 12;
    uint32_t approximate_width_mm : 12;   /* saturates at 4095 */
    uint32_t closest_angle_deg : 8;
    uint32_t left_clearance_mm : 12;
    uint32_t right_clearance_mm : 12;
    uint32_t span_deg : 8;
    uint32_t right_edge_mm : 12;          /* fine-scan edge rays */
    uint32_t left_edge_mm : 12;
    uint32_t right_edge_deg : 8;          /* (before correction) */
    uint8_t left_edge_deg;
    uint8_t obstacle_found : 1;
    uint8_t profile_valid : 1;
    uint8_t left_clearance_valid : 1;
    uint8_t right_clearance_valid : 1;
    uint8_t width_capped : 1;     /* an edge was at the end of the sweep:
                                     the width is a lower bound */
    uint8_t candidate_side : 2;   /* subsystem5_side_t, 3 values */
    uint8_t : 1;
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