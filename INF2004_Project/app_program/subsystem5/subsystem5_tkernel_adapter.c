/* micro T-Kernel application adapter. Add to team's existing task setup. */
#include "subsystem5_ultrasonic.h"
#include <tk/tkernel.h>

/* micro T-Kernel: smaller number means higher task priority.
 * Proposed value only; agree with VehicleController/MotionControl. */
#define subsystem5_TASK_PRIORITY   (10)
#define subsystem5_TASK_STACK      (2048)
#define subsystem5_POLL_PERIOD_MS  (2)

static ID s_subsystem5_task_id;

static void subsystem5_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    for (;;)
    {
        subsystem5_poll();
        (void)tk_dly_tsk(subsystem5_POLL_PERIOD_MS);
    }
}

/* Call once from the application's startup/usermain, AFTER the shared
 * GPIO dispatcher is registered. The callback must be non-blocking. */
ER subsystem5_tkernel_start(subsystem5_event_publish_fn publish, void *context)
{
    T_CTSK task_info =
    {
        .tskatr = TA_HLNG | TA_RNG3,
        .itskpri = subsystem5_TASK_PRIORITY,
        .stksz = subsystem5_TASK_STACK,
        .task = subsystem5_task
    };

    subsystem5_ultrasonic_init(publish, context);
    s_subsystem5_task_id = tk_cre_tsk(&task_info);
    if (s_subsystem5_task_id < E_OK)
    {
        return (ER)s_subsystem5_task_id;
    }
    return tk_sta_tsk(s_subsystem5_task_id, 0);
}
