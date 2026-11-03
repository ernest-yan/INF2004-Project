#include "terrain.h"
#include "mathf.h"   /* PORT: replaces <math.h> (no libm linked) */
#include <string.h>
#define B4_PITCH_START_DEG     (4.0f)  /* PROVISIONAL threshold. */
#define B4_PITCH_LEVEL_DEG     (2.0f)
#define B4_MIN_HUMP_MM         (5.0f)
#define B4_MAX_HUMP_MM         (150.0f)
#define B4_MAX_PROFILE_MM      (2000U)
#define B4_PI                  (3.14159265f)
static uint16_t b4_u16_round(float v)
{
    if (v <= 0.0f) { return 0U; }
    if (v >= 65535.0f) { return 65535U; }
    return (uint16_t)(v + 0.5f);
}
void b4_terrain_init(b4_terrain_t *model)
{
    if (model != NULL) { memset(model, 0, sizeof(*model)); }
}
bool b4_terrain_update(b4_terrain_t *model, float pitch_deg,
                       uint32_t step_mm)
{
    if (model == NULL) { return false; }
    /* A single huge encoder delta indicates slip/communication fault. */
    if (step_mm > 100U) { step_mm = 0U; }
    if (model->phase == B4_TERRAIN_LEVEL)
    {
        model->climb_samples = (pitch_deg > B4_PITCH_START_DEG &&
                                step_mm > 0U) ?
                               (uint8_t)(model->climb_samples + 1U) : 0U;
        if (model->climb_samples >= 2U)
        {
            model->phase = B4_TERRAIN_CLIMB;
            model->accumulated_z_mm = 0.0f;
            model->peak_z_mm = 0.0f;
            model->accumulated_path_mm = 0U;
            model->descent_samples = 0U;
            model->level_samples = 0U;
        }
        return false;
    }
    model->accumulated_path_mm += step_mm;
    if (model->accumulated_path_mm > B4_MAX_PROFILE_MM)
    {
        /* Incomplete or malformed profile: never report a fabricated hump. */
        const uint16_t previous_max = model->max_peak_mm;
        b4_terrain_init(model);
        model->max_peak_mm = previous_max;
        return false;
    }
    const float dz = (float)step_mm * b4_sinf(pitch_deg * B4_PI / 180.0f);
    model->accumulated_z_mm += dz;
    if (model->accumulated_z_mm > model->peak_z_mm)
    {
        model->peak_z_mm = model->accumulated_z_mm;
    }
    model->peak_mm = b4_u16_round(model->peak_z_mm);
    if (model->phase == B4_TERRAIN_CLIMB)
    {
        if (pitch_deg < -B4_PITCH_START_DEG)
        {
            model->descent_samples++;
            if (model->descent_samples >= 2U)
            {
                model->phase = B4_TERRAIN_DESCEND;
            }
        }
        else if (b4_fabsf(pitch_deg) <= B4_PITCH_LEVEL_DEG)
        {
            model->phase = B4_TERRAIN_TOP;
        }
    }
    else if (model->phase == B4_TERRAIN_TOP)
    {
        if (pitch_deg < -B4_PITCH_START_DEG)
        {
            model->descent_samples++;
            if (model->descent_samples >= 2U)
            {
                model->phase = B4_TERRAIN_DESCEND;
            }
        }
    }
    else if (model->phase == B4_TERRAIN_DESCEND)
    {
        model->level_samples = (b4_fabsf(pitch_deg) <= B4_PITCH_LEVEL_DEG) ?
                               (uint8_t)(model->level_samples + 1U) : 0U;
        if (model->level_samples >= 2U)
        {
            const bool complete = (model->peak_z_mm >= B4_MIN_HUMP_MM) &&
                                  (model->peak_z_mm <= B4_MAX_HUMP_MM);
            const uint16_t peak = model->peak_mm;
            const uint16_t max_peak = (peak > model->max_peak_mm && complete)
                                      ? peak : model->max_peak_mm;
            b4_terrain_init(model);
            model->max_peak_mm = max_peak;
            model->last_peak_mm = complete ? peak : 0U;
            return complete;
        }
    }
    return false;
}
