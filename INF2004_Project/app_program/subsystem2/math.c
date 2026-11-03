#include "math.h"
#include <limits.h>
uint32_t b2_mm_to_ticks(uint32_t mm, uint32_t tpr, uint32_t circ)
{
    if ((circ == 0U) || (tpr == 0U)) { return 0U; }
    const uint64_t result = (((uint64_t)mm * tpr) + (circ / 2U)) / circ;
    return (result > UINT32_MAX) ? UINT32_MAX : (uint32_t)result;
}
uint32_t b2_turn_to_ticks(uint32_t deg, uint32_t track, uint32_t tpr,
                          uint32_t circ)
{
    /* In-place differential turn: each wheel follows pi*track*deg/360. */
    const uint64_t length_mm =
        (((uint64_t)track * deg * 31416ULL) + 1800000ULL) / 3600000ULL;
    return b2_mm_to_ticks((uint32_t)length_mm, tpr, circ);
}
int32_t b2_ticks_to_mm(uint32_t ticks, uint32_t tpr, uint32_t circ)
{
    if (tpr == 0U) { return 0; }
    uint64_t result = ((uint64_t)ticks * circ) / tpr;
    if (result > INT32_MAX) { result = INT32_MAX; }
    return (int32_t)result;
}
int32_t b2_speed_mm_s(uint32_t ticks, uint32_t elapsed,
                      uint32_t tpr, uint32_t circ)
{
    if ((elapsed == 0U) || (tpr == 0U)) { return 0; }
    const uint64_t result = ((uint64_t)ticks * circ * 1000ULL) /
                            ((uint64_t)tpr * elapsed);
    return (result > INT32_MAX) ? INT32_MAX : (int32_t)result;
}
