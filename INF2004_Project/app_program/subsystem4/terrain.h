#ifndef B4_TERRAIN_H
#define B4_TERRAIN_H
#include <stdbool.h>
#include <stdint.h>
typedef enum
{
    B4_TERRAIN_LEVEL = 0, B4_TERRAIN_CLIMB, B4_TERRAIN_TOP,
    B4_TERRAIN_DESCEND
} b4_terrain_phase_t;
typedef struct
{
    b4_terrain_phase_t phase;
    float accumulated_z_mm;    /* Relative rise above local start (approximate). */
    float peak_z_mm;
    uint16_t peak_mm;
    uint16_t max_peak_mm;
    uint16_t last_peak_mm;
    uint32_t accumulated_path_mm;
    uint8_t climb_samples;
    uint8_t descent_samples;
    uint8_t level_samples;
} b4_terrain_t;
void b4_terrain_init(b4_terrain_t *model);
/* delta travel from validated encoder odometry, filtered pitch in degrees. */
/* Returns true only when a full up/down profile has been observed. */
bool b4_terrain_update(b4_terrain_t *model, float pitch_deg,
                       uint32_t travel_delta_mm);
#endif
