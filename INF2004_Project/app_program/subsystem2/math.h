#ifndef B2_MATH_H
#define B2_MATH_H
#include <stdint.h>
/* All distances in millimetres. Encoder scale is PROVISIONAL until measured. */
uint32_t b2_mm_to_ticks(uint32_t mm, uint32_t ticks_per_rev,
                        uint32_t circumference_mm);
uint32_t b2_turn_to_ticks(uint32_t turn_deg, uint32_t track_width_mm,
                          uint32_t ticks_per_rev, uint32_t circumference_mm);
int32_t b2_ticks_to_mm(uint32_t ticks, uint32_t ticks_per_rev,
                       uint32_t circumference_mm);
int32_t b2_speed_mm_s(uint32_t delta_ticks, uint32_t elapsed_ms,
                      uint32_t ticks_per_rev, uint32_t circumference_mm);
#endif
