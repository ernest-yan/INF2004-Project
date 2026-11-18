/*
 *	terrain.h
 *	Subsystem 4 -- hump detection and height.
 *
 *	Pure maths, no hardware (tested in subsystem4_test.c, S4-U03/U04/U13).
 *
 *	How it measures (peak-tilt method):
 *	A hump shorter than about two wheelbases does not lift the whole car;
 *	the car ROCKS over it. When the front reaches the top while the rear
 *	is still on the floor, the car is tilted by
 *	    sin(tilt) = hump height / wheelbase
 *	and when the rear reaches the top after the front has come down, it is
 *	tilted the same amount the other way. So
 *	    height = wheelbase * (sin(peak nose-up) + sin(peak nose-down)) / 2
 *	On this car the small front caster barely tilts it; the big rear drive
 *	wheels climbing give the clear, long nose-down half. So each half is
 *	used on its own when it is solid (tilted over >= 30 mm of driving),
 *	and the two are averaged when both are.
 *	The distance travelled between the two peaks is itself the wheelbase,
 *	which the hump summary reports as a cross-check (peak_gap_mm).
 *	The hump ends after 80 mm of travel with no clear tilt (a wheelbase +
 *	40 mm while only one direction has been seen), or when the car
 *	stops for 1 s (measured if both phases were seen, otherwise dropped).
 *	Only tilt seen while the wheels turn counts: picking up or moving a
 *	stopped car never changes a hump.
 *	Pulling away or braking tilts the car over only ~10 mm of travel, so
 *	it is never a solid half and never a hump.
 *	Accurate when the hump's half-length is no longer than the wheelbase.
 *
 *	The old method (distance x slope, added up) measured the AVERAGE height
 *	under the car: about 6 mm for the course's 10 mm hump.
 */
#ifndef B4_TERRAIN_H
#define B4_TERRAIN_H
#include <stdbool.h>
#include <stdint.h>
typedef enum
{
    B4_TERRAIN_LEVEL = 0, B4_TERRAIN_CLIMB, B4_TERRAIN_TOP,
    B4_TERRAIN_DESCEND
} b4_terrain_phase_t;          /* TOP is no longer used (kept for the wire). */
typedef struct
{
    b4_terrain_phase_t phase;  /* CLIMB = nose up, DESCEND = nose down. */
    float pitch_deg;           /* extra low-pass on the caller's pitch. */
    float max_pitch_deg;       /* most nose-up this hump. */
    float min_pitch_deg;       /* most nose-down this hump. */
    /* CR1: every travel counter is <= ~700 mm (a hump longer than
       B4_HUMP_MAX_PATH_MM = 600 mm is dropped and re-armed), so 16 bits;
       sample counters saturate at 255 (largest threshold is 100);
       heights saturate at 255 mm (B4_MAX_HUMP_MM is 150). */
    uint16_t path_mm;          /* travel since the hump started. */
    uint16_t max_at_mm;        /* path_mm when max_pitch_deg was seen. */
    uint16_t min_at_mm;        /* path_mm when min_pitch_deg was seen. */
    uint16_t up_mm;            /* travel while nose-up beyond threshold. */
    uint16_t down_mm;          /* travel while nose-down beyond threshold. */
    uint16_t clear_mm;         /* travel since the last clear tilt. */
    uint8_t start_samples;
    uint8_t still_samples;     /* consecutive samples with no travel. */
    uint8_t peak_mm;           /* height estimate so far, this hump. */
    uint8_t max_peak_mm;       /* highest completed hump. */
    uint8_t last_peak_mm;      /* last completed hump. */
    /* Summary of the last completed hump (console, for tuning). */
    int16_t last_max_cdeg;
    int16_t last_min_cdeg;
    uint16_t last_gap_mm;
} b4_terrain_t;
void b4_terrain_init(b4_terrain_t *model);
/* One sample (every 10 ms): pitch in degrees (+ = nose up), travel since the
   last sample from encoder odometry, and the car's wheelbase (distance
   between the front and rear ground contacts). Returns true once, when a
   complete hump (nose up AND nose down, then level or stopped) is measured;
   the height is then in last_peak_mm. */
bool b4_terrain_update(b4_terrain_t *model, float pitch_deg,
                       uint32_t travel_delta_mm, uint16_t wheelbase_mm);
#endif
