/*
 * Subsystem 5: HC-SR04 + SG90 servo, RP2040/Pico SDK.
 * Stop requests are advisory events for VehicleController; the obstacle bypass
 * drives through the S2 function API (IF-03).
 * NO motor PWM/encoder GPIOs are touched in this file.
 */
/*
 * PORT NOTES (micro T-Kernel template version).  Every change is marked "PORT:".
 * The ranging / scan / state-machine logic is unchanged.
 *   - no Pico SDK headers: GPIO and PWM go through libbsp
 *   - echo pulse timed by polling instead of a GPIO interrupt
 *   - interrupt masking replaced by a kernel mutex
 *   - <stddef.h>/<string.h> not used (size_t clash with the kernel headers)
 */
#include "ultrasonic.h"
#include "../app_hw.h"
#include "../subsystem2/motion.h"
#include "../subsystem4/mathf.h"   /* b4_sinf/b4_atan2f: bypass pose (no libm) */
#include "../subsystem3/line_barcode.h"
#include "../subsystem1/subsystem1.h"  /* s1_mission_go(): STOP cancels a bypass */

/* PORT: replaces time_us_32().  RP2040 TIMER counts microseconds; TIMERAWL
 * (offset 0x28) is a non-latching read of the low 32 bits. */
#define subsystem5_TIMERAWL   (*(volatile uint32_t *)0x40054028u)

static inline uint32_t time_us_32(void)
{
    return subsystem5_TIMERAWL;
}

/* PORT: replaces busy_wait_us_32(). */
static void subsystem5_busy_wait_us(uint32_t us)
{
    const uint32_t start = time_us_32();
    while ((uint32_t)(time_us_32() - start) < us)
    {
    }
}

/* PORT: the servo is on GP12 = PWM slice 6, channel A.  libbsp can set the
 * wrap value and the compare value but NOT the clock divider, so the divider
 * register is written directly.  The system clock is 125 MHz (cpu_clock.c);
 * DIV register = integer part << 4 | fraction, so 125 << 4 gives a 1 MHz tick:
 * 1 count = 1 us, wrap 19999 = 20 ms (50 Hz), compare value = pulse in us. */
#define subsystem5_PWM_CH_DIV   (*(volatile uint32_t *)(0x40050000u + (6u * 0x14u) + 0x04u))
#define subsystem5_PWM_DIV_1MHZ (125u << 4)

/* PORT: mutex replaces save_and_disable_interrupts() (that only stops one
 * core, and this build runs two). */
static ID s_mtx = 0;

static void subsystem5_lock(void)
{
    if (s_mtx > 0)
    {
        (void)tk_loc_mtx(s_mtx, TMO_FEVR);
    }
}

static void subsystem5_unlock(void)
{
    if (s_mtx > 0)
    {
        (void)tk_unl_mtx(s_mtx);
    }
}

#define subsystem5_PING_GAP_US          (65000U) /* Datasheet: >60 ms. */
#define subsystem5_ECHO_TIMEOUT_US      (30000U)
#define subsystem5_SERVO_PERIOD_US      (20000U) /* 50 Hz. */
#define subsystem5_SERVO_MIN_US         (1000U)  /* Calibrate for actual SG90. */
#define subsystem5_SERVO_MAX_US         (2000U)
#define subsystem5_SERVO_MIN_DEG        (30U)
#define subsystem5_SERVO_MAX_DEG        (150U)
#define subsystem5_SERVO_WAIT_BASE_MS   (60U)
#define subsystem5_SERVO_WAIT_PER_DEG   (3U)     /* Conservative starting estimate. */
#define subsystem5_STOP_TIMEOUT_MS      (1500U)
#define subsystem5_MAX_INVALID_MONITOR  (3U)
#define subsystem5_MAX_RESCANS          (1U)
/* INTEGRATION FIX: an echo that rose but never came back in range is open
 * space, not a fault.  It is reported as a valid reading this far away (the
 * HC-SR04's 4 m limit), beyond every obstacle and clearance limit. */
#define subsystem5_FAR_MM               (4000U)
/* INTEGRATION FIX: close readings in a row before a stop, so one stray echo
 * does not stop the car.  Under subsystem5_URGENT_MM one reading is enough. */
#define subsystem5_CLOSE_READINGS       (2U)

#define subsystem5_ECHO_WAIT_RISE       (1U)
#define subsystem5_ECHO_WAIT_FALL       (2U)
#define subsystem5_ECHO_COMPLETE        (3U)

/* ECHO is timed by polling at the lowest task priority, so a higher-priority
 * task (WiFi start-up, telemetry, motion) can run in the middle of a ping.
 * A gap this long between two polls could hide a whole echo pulse (the
 * shortest usable one is 116 us) or stretch its width, so such a ping is
 * thrown away and repeated instead of being counted as a sensor failure.
 * After this many disturbed pings in a row the result is used anyway, so a
 * permanently busy CPU still ends in a fault rather than silence. */
#define subsystem5_POLL_GAP_US          (100U)
#define subsystem5_MAX_DISTURBED        (5U)

static const uint8_t s_coarse_angles[subsystem5_COARSE_COUNT] =
{
    30U, 60U, 90U, 120U, 150U
};

/* PORT: echo variables are now touched only by the poll task (no interrupt). */
static volatile uint32_t s_echo_rise_us;
static volatile uint32_t s_echo_pulse_us;
static uint8_t s_disturbed_run;     /* disturbed pings dropped in a row */

/* CR1: flags and small state machines written by the poll task ONLY, so a
 * bit-field read-modify-write here can never race (single writer). The
 * command flags further down are written by OTHER tasks and stay one byte
 * each: a byte store needs no lock, a shared bit-field byte would. */
static struct
{
    uint16_t echo_phase : 2;        /* subsystem5_ECHO_*, 0..3 */
    uint16_t echo_disturbed : 1;    /* this ping saw a poll gap > POLL_GAP_US */
    uint16_t range_active : 1;
    uint16_t bp_phase : 3;          /* BP_OUT..BP_TURN, 5 values */
    uint16_t bp_left : 1;           /* bypass on the left (was subsystem5_bp_dir() = -1) */
    uint16_t bp_seen : 1;           /* PASS: obstacle seen beside the car */
    uint16_t : 7;
} s_poll;

/* Poll-task-owned state; other tasks use command flags and snapshot API. */
static subsystem5_status_t s_status;
static subsystem5_status_t s_snapshot;
static subsystem5_scan_samples_t s_samples;
static subsystem5_event_publish_fn s_publish;
static void *s_publish_context;
static uint8_t s_scan_index;
static uint8_t s_scan_retries;
#if APP_MISSION_ENABLE
static uint8_t s_bp_probe;           /* 1 = next bypass measures the edge itself */
#endif
static uint8_t s_reverse_count;
static uint8_t s_monitor_invalid;
static uint8_t s_close_count;
static uint8_t s_servo_angle;
static uint8_t s_fine_angles[subsystem5_FINE_COUNT];
static uint32_t s_range_started_us;
static uint32_t s_last_ping_us;
static uint32_t s_servo_ready_us;
static uint32_t s_stop_request_us;
static volatile uint8_t s_stopped_confirmed;
static volatile uint8_t s_stationary_scan_requested;
static volatile uint8_t s_line_recovered_ack;
static volatile uint8_t s_fault_reset_request;

static void subsystem5_snapshot_update(void)
{
    subsystem5_lock();      /* PORT: was save_and_disable_interrupts() */
    s_snapshot = s_status;
    subsystem5_unlock();
}

static uint32_t subsystem5_now_us(void)
{
    return time_us_32();
}

static uint32_t subsystem5_elapsed(uint32_t start, uint32_t end)
{
    return end - start; /* Wrap-safe for bounded intervals <2^31 us. */
}

static void subsystem5_servo_set(uint8_t angle_deg)
{
    uint8_t previous = s_servo_angle;
    uint8_t difference;
    uint32_t pulse_us;
    uint32_t settle_ms;

    if (angle_deg < subsystem5_SERVO_MIN_DEG)
    {
        angle_deg = subsystem5_SERVO_MIN_DEG;
    }
    else if (angle_deg > subsystem5_SERVO_MAX_DEG)
    {
        angle_deg = subsystem5_SERVO_MAX_DEG;
    }
    else
    {
        /* Within the deliberately limited scan arc. */
    }

    difference = (angle_deg > previous) ?
                 (angle_deg - previous) : (previous - angle_deg);
    pulse_us = subsystem5_SERVO_MIN_US +
               ((uint32_t)angle_deg *
                (subsystem5_SERVO_MAX_US - subsystem5_SERVO_MIN_US)) / 180U;
    (void)pwm_set_cc(subsystem5_SERVO_GPIO, pulse_us);	/* PORT: was pwm_set_chan_level() */

    settle_ms = subsystem5_SERVO_WAIT_BASE_MS +
                ((uint32_t)difference * subsystem5_SERVO_WAIT_PER_DEG);
    s_servo_ready_us = subsystem5_now_us() + (settle_ms * 1000U);
    s_servo_angle = angle_deg;
}

static void subsystem5_send_event(subsystem5_event_type_t type, subsystem5_fault_t fault)
{
    subsystem5_event_t event;
    bool accepted = false;

    subsystem5_memset(&event, 0, (int)sizeof(event));
    event.type = (uint16_t)type;
    event.fault = (uint8_t)fault;
    event.front_mm = s_status.front_mm;
    event.profile = s_status.profile;

    if (s_publish != NULL)
    {
        accepted = s_publish(&event, s_publish_context);
    }

    /* A missed critical event leaves a latched fault and stop request.
     * VehicleController MUST ALSO check the snapshot independently. */
    if (!accepted)
    {
        s_status.fault_mask |= subsystem5_FAULT_BIT(subsystem5_FAULT_EVENT_DELIVERY);
        s_status.stop_required = 1U;
        s_status.state = subsystem5_STATE_FAULT;
    }
}

static void subsystem5_fault(subsystem5_fault_t fault)
{
    s_status.stop_required = 1U;
    s_status.fault_mask |= subsystem5_FAULT_BIT(fault);
    s_status.state = subsystem5_STATE_FAULT;
    subsystem5_send_event(subsystem5_EVENT_FAULT, fault);
}

/* PORT: replaces the GPIO edge interrupt.  Polls the configured ECHO GPIO with 1 us time stamps
 * until the echo pulse has risen and fallen, or the timeout passes.  Same
 * state machine as the old subsystem5_echo_gpio_irq(), driven by the pin
 * level instead of edge events. */
static void subsystem5_echo_measure(void)
{
    /* From the ping's start, so a preemption around the TRIG pulse counts too
     * (the 12 us TRIG pulse itself is well under POLL_GAP_US). */
    uint32_t last_poll = s_range_started_us;

    for (;;)
    {
        const uint32_t t = subsystem5_now_us();
        const UINT level = gpio_get_val(subsystem5_ECHO_GPIO);

        if (subsystem5_elapsed(last_poll, t) > subsystem5_POLL_GAP_US)
        {
            s_poll.echo_disturbed = 1U;  /* preempted: an edge may have been missed */
        }
        last_poll = t;

        if (subsystem5_elapsed(s_range_started_us, t) >= subsystem5_ECHO_TIMEOUT_US)
        {
            return; /* No complete echo: subsystem5_ping_result() reports invalid. */
        }

        if ((s_poll.echo_phase == subsystem5_ECHO_WAIT_RISE) && (level != 0U))
        {
            s_echo_rise_us = t;
            s_poll.echo_phase = subsystem5_ECHO_WAIT_FALL;
        }
        else if ((s_poll.echo_phase == subsystem5_ECHO_WAIT_FALL) && (level == 0U))
        {
            s_echo_pulse_us = subsystem5_elapsed(s_echo_rise_us, t);
            s_poll.echo_phase = subsystem5_ECHO_COMPLETE;
            return;
        }
        else
        {
            /* Still waiting for an edge. */
        }
    }
}

/* Returns true only if a new measurement was initiated. */
static bool subsystem5_ping_begin(uint32_t now_us)
{
    if ((s_poll.range_active != 0U) ||
        (subsystem5_elapsed(s_last_ping_us, now_us) < subsystem5_PING_GAP_US))
    {
        return false;
    }

    s_poll.echo_phase = subsystem5_ECHO_WAIT_RISE;
    s_echo_pulse_us = 0U;
    s_echo_rise_us = 0U;
    s_poll.echo_disturbed = 0U;

    s_poll.range_active = 1U;
    s_range_started_us = subsystem5_now_us();
    s_last_ping_us = s_range_started_us;

    (void)gpio_set_val(subsystem5_TRIG_GPIO, 1U);   /* PORT: was gpio_put() */
    subsystem5_busy_wait_us(12U);                   /* Only the 12-us TRIG pulse is busy-waited. */
    (void)gpio_set_val(subsystem5_TRIG_GPIO, 0U);

    subsystem5_echo_measure();                      /* PORT: replaces the echo IRQ */
    return true;
}

/* Returns true when completed (valid or invalid), false if still pending. */
static bool subsystem5_ping_result(uint32_t now_us, subsystem5_sample_t *result)
{
    uint32_t pulse_us = 0U;
    uint8_t phase;

    if ((s_poll.range_active == 0U) || (result == NULL))
    {
        return false;
    }

    /* PORT: no interrupt-masked copy needed; only this task touches these. */
    phase = s_poll.echo_phase;
    if (phase == subsystem5_ECHO_COMPLETE)
    {
        pulse_us = s_echo_pulse_us;
        s_poll.echo_phase = 0U;
    }
    else if (subsystem5_elapsed(s_range_started_us, now_us) >= subsystem5_ECHO_TIMEOUT_US)
    {
        s_poll.echo_phase = 0U;
    }
    else
    {
        /* Still within the timeout window. */
    }

    if ((phase != subsystem5_ECHO_COMPLETE) &&
        (subsystem5_elapsed(s_range_started_us, now_us) < subsystem5_ECHO_TIMEOUT_US))
    {
        return false;
    }

    if ((s_poll.echo_disturbed != 0U) && (s_disturbed_run < subsystem5_MAX_DISTURBED))
    {
        /* Not a sample: drop it.  The caller sees "still pending" with no
         * range active, so it simply starts the next ping. */
        s_disturbed_run++;
        s_poll.range_active = 0U;
        return false;
    }
    s_disturbed_run = 0U;

    result->valid = 0U;
    result->distance_mm = subsystem5_UNKNOWN_MM;
    if ((phase == subsystem5_ECHO_COMPLETE) &&
        (pulse_us >= 116U) && (pulse_us <= 23200U))
    {
        /* Datasheet: distance_cm = echo_us / 58; integer mm. */
        result->distance_mm = (uint16_t)((pulse_us * 10U) / 58U);
        result->valid = 1U;
    }
    else if ((phase == subsystem5_ECHO_WAIT_FALL) ||
             ((phase == subsystem5_ECHO_COMPLETE) && (pulse_us > 23200U)))
    {
        /* INTEGRATION FIX: the echo rose, so the sensor is alive, but nothing
         * came back in range: open space.  Only an echo pin that never rises
         * (e.g. unplugged) stays invalid and can fault. */
        result->distance_mm = subsystem5_FAR_MM;
        result->valid = 1U;
    }
    else
    {
        /* Echo never rose, or a pulse too short to use. */
    }
    s_poll.range_active = 0U;
    return true;
}

static void subsystem5_abort_pending_echo(void)
{
    s_poll.echo_phase = 0U;      /* PORT: no interrupt masking needed */
    s_poll.range_active = 0U;
}

/* TRUE when a close reading should stop the car (see the monitor state). */
static bool subsystem5_watch_active(void)
{
#if APP_MISSION_ENABLE
    return s3_line_following() ? true : false;
#else
    return true;    /* nothing drives in this build: keep the old behaviour */
#endif
}

static void subsystem5_reset_for_monitor(void)
{
    s_status.fault_mask = 0U;
    s_status.stop_required = 0U;
    s_status.scan_complete = 0U;
    s_status.state = subsystem5_STATE_MONITOR;
    s_status.bypass_phase = 0U;
    s_status.front_valid = 0U;
    s_monitor_invalid = 0U;
    s_close_count = 0U;
    subsystem5_abort_pending_echo();
    s_stopped_confirmed = 0U;
    s_stationary_scan_requested = 0U;
    subsystem5_servo_set(90U);
}

static void subsystem5_scan_start(void)
{
    subsystem5_memset(&s_samples, 0, (int)sizeof(s_samples));
    s_scan_index = 0U;
    s_status.scan_complete = 0U;
    subsystem5_memset(&s_status.profile, 0, (int)sizeof(s_status.profile));
    s_status.state = subsystem5_STATE_COARSE;
    subsystem5_servo_set(s_coarse_angles[0]);
}

static uint8_t subsystem5_clamp_angle(int16_t angle)
{
    if (angle < (int16_t)subsystem5_SERVO_MIN_DEG)
    {
        return subsystem5_SERVO_MIN_DEG;
    }
    if (angle > (int16_t)subsystem5_SERVO_MAX_DEG)
    {
        return subsystem5_SERVO_MAX_DEG;
    }
    return (uint8_t)angle;
}

/* A finished scan: hand the profile to the vehicle controller / bypass. */
static void subsystem5_profile_ready(void)
{
    s_status.scan_complete = 1U;
    s_status.state = subsystem5_STATE_PROFILE_READY;
    subsystem5_servo_set(90U);
    subsystem5_send_event(subsystem5_EVENT_PROFILE_READY, subsystem5_FAULT_NONE);
}

/* 19 Nov: the sweep cannot be trusted (servo not turning, or rays with no
 * echo) even after a rescan. The car is not stuck, it is only blind to the
 * sides: if the front sensor still sees the obstacle, bypass it anyway and
 * let the bypass measure the obstacle's edge with the front sensor as the
 * car curves out (the car body does the sweep). Nothing in front: carry
 * on. SAFE_STOP (fault) only when there is no mission to drive. */
static void subsystem5_scan_untrusted(void)
{
#if APP_MISSION_ENABLE
    const subsystem5_sample_t *right = &s_samples.coarse[0];   /* 30 deg */
    const subsystem5_sample_t *left =
        &s_samples.coarse[subsystem5_COARSE_COUNT - 1U];         /* 150 deg */

    if (s1_mission_go() && (s_status.front_valid != 0U) &&
        (s_status.front_mm > subsystem5_OBSTACLE_ZONE_MM))
    {
        tm_printf((UB*)"[S5] scan not usable and nothing ahead: carrying on\n");
        subsystem5_reset_for_monitor();
        return;
    }
    if (s1_mission_go() && (s_status.front_valid != 0U))
    {
        subsystem5_profile_t p;

        subsystem5_memset(&p, 0, (int)sizeof(p));
        p.profile_valid = 1U;
        p.obstacle_found = 1U;
        p.closest_mm = s_status.front_mm;
        p.closest_angle_deg = 90U;
        /* The left (team choice), unless its side ray says it has no room
           and the right ray reads CLEARLY further (more than FACE_TOL).
           20 Nov 14:05/14:08: at ~20 cm both side rays hear the box itself
           (e.g. 18 vs 17 cm), and a 1-2 mm difference sent the car right
           every other run (left, right, left, right). */
        p.candidate_side = ((left->valid != 0U) && (right->valid != 0U) &&
                            (left->distance_mm < subsystem5_SIDE_MIN_MM) &&
                            (right->distance_mm >
                             (left->distance_mm + subsystem5_FACE_TOL_MM))) ?
                           subsystem5_SIDE_RIGHT : subsystem5_SIDE_LEFT;
        s_status.profile = p;
        s_status.profile_count++;
        s_bp_probe = 1U;
        tm_printf((UB*)"[S5] scan not usable (servo?): probing round on the %s, "
                  "obstacle %u mm ahead\n",
                  (p.candidate_side == subsystem5_SIDE_LEFT) ? "left" : "right",
                  (unsigned)p.closest_mm);
        subsystem5_profile_ready();
        return;
    }
#endif
    subsystem5_fault(subsystem5_FAULT_SCAN_INVALID);
}

static void subsystem5_prepare_fine(void)
{
    uint16_t closest_mm = UINT16_MAX;
    uint16_t farthest_mm = 0U;
    uint8_t focus = 90U;
    uint8_t index;
    uint8_t invalid = 0U;

    for (index = 0U; index < subsystem5_COARSE_COUNT; index++)
    {
        const subsystem5_sample_t *point = &s_samples.coarse[index];
        if (point->valid == 0U)
        {
            invalid++;
        }
        else if (point->distance_mm > farthest_mm)
        {
            farthest_mm = point->distance_mm;
        }
        else
        {
            /* not the farthest */
        }
        if ((point->valid != 0U) &&
            (point->distance_mm <= subsystem5_OBSTACLE_ZONE_MM) &&
            (point->distance_mm < closest_mm))
        {
            closest_mm = point->distance_mm;
            focus = (uint8_t)point->angle_deg;
        }
    }

    /* 20 Nov, diagnostics: what each coarse ray read, for telemetry. */
    for (index = 0U; index < subsystem5_COARSE_COUNT; index++)
    {
        const subsystem5_sample_t *point = &s_samples.coarse[index];

        s_status.coarse_cm[index] =
            (point->valid == 0U) ? 255U :
            ((point->distance_mm >= subsystem5_FAR_MM) ? 254U :
             (uint8_t)(((point->distance_mm / 10U) > 253U) ? 253U :
                       (point->distance_mm / 10U)));
    }
    if ((closest_mm == UINT16_MAX) && (invalid == 0U))
    {
        /* INTEGRATION FIX: every ray answered and nothing is there (the
         * obstacle moved, or it was a stray echo): carry on, not a fault. */
        tm_printf((UB*)"[S5] scan found nothing, carrying on\n");
        subsystem5_reset_for_monitor();
        return;
    }
    /* 19 Nov: an obstacle in range on EVERY ray from 30 to 150 degrees,
     * all within FACE_TOL of each other, cannot be a real object: a flat
     * face reads twice as far at 30/150 as straight on. It means the servo
     * is not turning (in the logs from 19:39, all scans read "nearest =
     * left = right", with the power bank running down). Rescan, then
     * probe round with the front sensor (subsystem5_scan_untrusted). */
    if ((closest_mm != UINT16_MAX) && (invalid == 0U) &&
        ((farthest_mm - closest_mm) <= subsystem5_FACE_TOL_MM))
    {
        tm_printf((UB*)"[S5] every ray reads %u-%u mm: servo not sweeping? "
                  "(check its power)\n",
                  (unsigned)closest_mm, (unsigned)farthest_mm);
        closest_mm = UINT16_MAX;
        invalid = 1U;                    /* treated as an invalid scan */
    }
    if (closest_mm == UINT16_MAX)
    {
        if (s_scan_retries < subsystem5_MAX_RESCANS)
        {
            s_scan_retries++;
            subsystem5_scan_start();
        }
        else
        {
            subsystem5_scan_untrusted();
        }
        return;
    }

    /* 15 rays, 5 degrees apart, centred on the closest coarse ray; the
       window is shifted at the arc limits so every angle is reachable. */
    {
        const uint8_t span = (uint8_t)((subsystem5_FINE_COUNT - 1U) *
                                       subsystem5_FINE_STEP_DEG);
        const uint8_t last_start = (uint8_t)(subsystem5_SERVO_MAX_DEG - span);
        const uint8_t first = subsystem5_clamp_angle(
            (int16_t)focus - (int16_t)(span / 2U));
        const uint8_t start = (first > last_start) ? last_start : first;
        for (index = 0U; index < subsystem5_FINE_COUNT; index++)
        {
            s_fine_angles[index] =
                (uint8_t)(start + (index * subsystem5_FINE_STEP_DEG));
        }
    }

    s_scan_index = 0U;
    s_status.state = subsystem5_STATE_FINE;
    subsystem5_servo_set(s_fine_angles[0]);
}

#if APP_MISSION_ENABLE
static void subsystem5_reverse_begin(void);
#endif

static void subsystem5_scan_finish(void)
{
    s_status.profile = subsystem5_profile_analyze(&s_samples,
                                          subsystem5_OBSTACLE_ZONE_MM,
                                          subsystem5_SIDE_CLEAR_MM);
    if ((s_status.profile.profile_valid != 0U) &&
        (s_status.profile.obstacle_found == 0U))
    {
        /* INTEGRATION FIX: a complete scan with nothing in it carries on. */
        tm_printf((UB*)"[S5] scan found nothing, carrying on\n");
        subsystem5_reset_for_monitor();
        return;
    }
    if (s_status.profile.profile_valid == 0U)
    {
        if (s_scan_retries < subsystem5_MAX_RESCANS)
        {
            s_scan_retries++;
            subsystem5_scan_start();
        }
        else
        {
            subsystem5_scan_untrusted();
        }
        return;
    }

    {
        const subsystem5_profile_t *p = &s_status.profile;
        s_status.profile_count++;
        tm_printf((UB*)"[S5] profile: closest %u mm @ %u deg, width %u mm%s "
                  "(edges %u deg/%u mm .. %u deg/%u mm), clear L %u R %u mm\n",
                  (unsigned)p->closest_mm, (unsigned)p->closest_angle_deg,
                  (unsigned)p->approximate_width_mm,
                  p->width_capped ? "+ (wider than the sweep)" : "",
                  (unsigned)p->right_edge_deg, (unsigned)p->right_edge_mm,
                  (unsigned)p->left_edge_deg, (unsigned)p->left_edge_mm,
                  p->left_clearance_valid ? (unsigned)p->left_clearance_mm : 0U,
                  p->right_clearance_valid ? (unsigned)p->right_clearance_mm : 0U);
    }

    if (s_status.profile.candidate_side == subsystem5_SIDE_NONE)
    {
        /* 19 Nov: go around on the more open side if it has room for the
           car, rather than reversing. */
        const subsystem5_profile_t *p = &s_status.profile;
        const uint16_t left_mm = (p->left_clearance_valid != 0U) ?
                                 (uint16_t)p->left_clearance_mm : 0U;
        const uint16_t right_mm = (p->right_clearance_valid != 0U) ?
                                  (uint16_t)p->right_clearance_mm : 0U;
        const uint16_t best_mm = (left_mm >= right_mm) ? left_mm : right_mm;

        if (best_mm >= subsystem5_SIDE_MIN_MM)
        {
            s_status.profile.candidate_side = (left_mm >= right_mm) ?
                subsystem5_SIDE_LEFT : subsystem5_SIDE_RIGHT;
            tm_printf((UB*)"[S5] no side has %u mm, passing on the %s (%u mm)\n",
                      (unsigned)subsystem5_SIDE_CLEAR_MM,
                      (left_mm >= right_mm) ? "left" : "right",
                      (unsigned)best_mm);
        }
    }

#if subsystem5_BYPASS_PREFER_LEFT
    /* Team choice: pass on the left whenever the left has room for the car,
       even if the right has more. */
    if ((s_status.profile.candidate_side == subsystem5_SIDE_RIGHT) &&
        (s_status.profile.left_clearance_valid != 0U) &&
        (s_status.profile.left_clearance_mm >= subsystem5_SIDE_MIN_MM))
    {
        s_status.profile.candidate_side = subsystem5_SIDE_LEFT;
        tm_printf((UB*)"[S5] left has %u mm: passing on the left (preferred)\n",
                  (unsigned)s_status.profile.left_clearance_mm);
    }
#endif

    if (s_status.profile.candidate_side == subsystem5_SIDE_NONE)
    {
#if APP_MISSION_ENABLE
        if ((s_reverse_count < subsystem5_MAX_REVERSES) && s1_mission_go())
        {
            s_reverse_count++;
            subsystem5_reverse_begin();
            return;
        }
#endif
        tm_printf((UB*)"[S5] no side has %u mm clear: stopped\n",
                  (unsigned)subsystem5_SIDE_CLEAR_MM);
        subsystem5_fault(subsystem5_FAULT_NO_SAFE_SIDE);
        return;
    }

    subsystem5_profile_ready();
}

static void subsystem5_scan_step(uint32_t now_us)
{
    subsystem5_sample_t result;
    subsystem5_sample_t *destination;
    const uint8_t count = (s_status.state == subsystem5_STATE_COARSE) ?
                          subsystem5_COARSE_COUNT : subsystem5_FINE_COUNT;
    const uint8_t current_angle = (s_status.state == subsystem5_STATE_COARSE) ?
        s_coarse_angles[s_scan_index] : s_fine_angles[s_scan_index];

    if ((s_poll.range_active == 0U) &&
        ((int32_t)(now_us - s_servo_ready_us) >= 0))
    {
        (void)subsystem5_ping_begin(now_us);
    }

    result.angle_deg = current_angle;
    if (!subsystem5_ping_result(now_us, &result))
    {
        return;
    }

    destination = (s_status.state == subsystem5_STATE_COARSE) ?
                  &s_samples.coarse[s_scan_index] :
                  &s_samples.fine[s_scan_index];
    *destination = result;
    s_scan_index++;
    if (s_status.state == subsystem5_STATE_COARSE)
    {
        s_samples.coarse_count = s_scan_index;
    }
    else
    {
        s_samples.fine_count = s_scan_index;
    }

    if (s_scan_index < count)
    {
        const uint8_t next_angle = (s_status.state == subsystem5_STATE_COARSE) ?
            s_coarse_angles[s_scan_index] : s_fine_angles[s_scan_index];
        subsystem5_servo_set(next_angle);
    }
    else if (s_status.state == subsystem5_STATE_COARSE)
    {
        subsystem5_prepare_fine();
    }
    else
    {
        subsystem5_scan_finish();
    }
}

#if APP_MISSION_ENABLE
/* Obstacle bypass (19 Nov rewrite), closed-loop on the car's own position.
 *
 * The wheel encoders give a live pose from the start of the bypass: x along
 * the original line, y sideways (+ = left), and the heading. Every phase
 * steers toward a target worked out from the measured obstacle profile or
 * from where the line sensors last saw the line; no turn angle or encoder
 * count is fixed in advance. The car never pivots: on 19 Nov every pivot
 * stalled a wheel, latched S2's brake and ended the bypass. It only drives
 * forward on a curve, or straight back. Before changing direction it waits
 * until both wheels have really stopped, so the single-channel encoders
 * count each edge in the right direction.
 *
 *   OUT     steer into the lane beside the obstacle (its edge + half the car
 *           + a margin), aiming to be in it when the front of the car
 *           reaches the obstacle's face.
 *   PASS    hold the lane, sensor looking at the obstacle, until it has been
 *           passed and the rear of the car is clear of it.
 *   RETURN  steer back to the line, aiming a look-ahead distance down it,
 *           at a steady BYPASS_APPROACH_DEG toward it: if the encoders
 *           put the line in the wrong place the car still reaches it.
 *   ALIGN   slowly, steered by the IR sensors like S3's line following:
 *           centre on (left off) = on the line, straighten up; left on =
 *           the line is to the left, steer left; neither = steer back to
 *           the side it was last seen on. Done when the centre sensor has
 *           stayed on the line, left off, for SENSOR_TO_AXLE of travel: the
 *           car is parallel with the centre sensor over the tape. Then S3
 *           takes over while the car is still moving.
 * 20 Nov: no reversing any more. Every back-up on the car failed to find
 * the line again and ended in SAFE_STOP; once the car has found the line it
 * never stops for alignment: if ALIGN runs long, S3's own search takes
 * over.
 * Where the line is comes from the sensors: when the centre sensor reaches
 * the tape, the line's middle is half a tape width further on (line width:
 * Barcode/Line Specification, 03 in the project references).
 * Accuracy depends on S2's calibration (motion_cal.h): wheel circumference
 * for distance, wheel track for heading. */
#define BP_OUT     (0U)
#define BP_PASS    (1U)
#define BP_RETURN  (2U)
#define BP_ALIGN   (3U)
#define BP_TURN    (4U)   /* turn on the spot first (20 Nov) */
#define BP_LOOK    (5U)   /* look both ways for the line (20 Nov) */

#define S5_RAD_PER_DEG  (0.01745329f)
#define S5_DEG_PER_RAD  (57.2957795f)

static const char *const s_bp_name[] =
{
    "out", "pass", "return", "align", "turn", "look"
};

static b2_goal_t s_bypass_goal;      /* used by the reverse step */

/* Pose of the drive axle centre, from the start of the bypass. */
static struct
{
    float x_mm;
    float y_mm;
    float heading_rad;               /* + = turned left of the line */
    float dist_l_mm;                 /* signed wheel travel since the start */
    float dist_r_mm;
    float path_mm;                   /* total travel, either direction */
    uint32_t last_l;                 /* encoder counts at the last update */
    uint32_t last_r;
} s_pose;

static float s_bp_lane_mm;           /* target lane, signed (+ = left) */
static float s_bp_edge_mm;           /* obstacle edge, distance toward the
                                        passing side (+ = that side) */
static float s_bp_back_mm;           /* furthest obstacle point seen (x) */
static uint8_t s_bp_fan_i;           /* position in the sweep fan */
static uint8_t s_bp_turn_stopping;   /* TURN: done, waiting for the wheels */
static float s_bp_expect_mm;        /* RETURN: travel to reach the line */
static float s_bp_look_h0;          /* LOOK: heading when the look began */
static int8_t s_bp_look_dir;        /* LOOK: +1 turning left, -1 right */
static uint8_t s_bp_looks;          /* LOOKs this bypass (ALIGN: at most 2) */
static uint8_t s_bp_look_step;      /* LOOK: 0 stopping, 1 first way,
                                        2 the other way */
static float s_bp_face_mm;           /* x where the car's front meets the face */
static float s_bp_phase_path_mm;     /* path at the start of the current phase */
static float s_bp_line_y_mm;         /* where the line is: 0, then the
                                        centre sensor's last sighting */
static float s_bp_seen_path_mm;      /* path when a sensor last saw the line */
static float s_bp_prev_sensor_y_mm;  /* centre sensor y at the last poll */
static uint8_t s_bp_prev_centre;     /* centre sensor on the line then */
static uint8_t s_bp_clear_count;
static uint8_t s_bp_al_step;         /* ALIGN: 0 drive on, 1 stopping,
                                        2 turning, 3 stopping after a turn */
static uint8_t s_bp_al_turns;        /* ALIGN: turns on the spot so far */
static int8_t s_bp_al_dir;           /* ALIGN: +1 turning left, -1 right */
static float s_bp_al_heading;        /* ALIGN: heading held while driving on */
static float s_bp_al_start_mm;       /* ALIGN: path when it began */
static float s_bp_al_turn_h0;        /* ALIGN: heading when a turn began */
static uint32_t s_bp_started_us;
static uint32_t s_bp_next_cmd_us;
static int32_t s_bp_cmd_l;           /* last wheel requests sent */
static int32_t s_bp_cmd_r;
static int8_t s_bp_dir_l;            /* direction the wheel was last driven */
static int8_t s_bp_dir_r;

/* +1 pass on the left, -1 on the right (CR1: stored as 1 bit). */
static float subsystem5_bp_side(void)
{
    return (s_poll.bp_left != 0U) ? 1.0f : -1.0f;
}

/* cos for |a| <= pi (b4_sinf is valid for |x| <= pi). */
static float subsystem5_cos_rad(float a)
{
    return b4_sinf(B4M_HALF_PI - b4_fabsf(a));
}

static float subsystem5_clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static int32_t subsystem5_clamp_i(int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static void subsystem5_drive(int32_t left_mm_s, int32_t right_mm_s)
{
    b2_command_t cmd;

    subsystem5_memset(&cmd, 0, (int)sizeof(cmd));
    cmd.type = B2_CMD_SPEED;
    cmd.left_mm_s = (int16_t)left_mm_s;
    cmd.right_mm_s = (int16_t)right_mm_s;
    s_bp_cmd_l = left_mm_s;
    s_bp_cmd_r = right_mm_s;
    if (left_mm_s != 0) { s_bp_dir_l = (left_mm_s < 0) ? -1 : 1; }
    if (right_mm_s != 0) { s_bp_dir_r = (right_mm_s < 0) ? -1 : 1; }
    (void)b2_submit_command(&cmd);
}

static void subsystem5_pose_reset(void)
{
    b2_status_t st;

    b2_get_status(&st);
    subsystem5_memset(&s_pose, 0, (int)sizeof(s_pose));
    s_pose.last_l = st.left_encoder_edges;
    s_pose.last_r = st.right_encoder_edges;
    s_bp_dir_l = 1;
    s_bp_dir_r = 1;
}

/* Dead reckoning. Each encoder edge counts in the direction the wheel was
 * last driven (S2 does the same); the bypass only changes direction once
 * the wheels have stopped. The heading comes from the totals since the
 * start, so rounding does not build up. */
static void subsystem5_pose_update(void)
{
    const float mm_l = (float)B2_CAL_WHEEL_CIRCUMFERENCE_MM /
                       (float)B2_CAL_TICKS_PER_REV_LEFT;
    const float mm_r = (float)B2_CAL_WHEEL_CIRCUMFERENCE_MM /
                       (float)B2_CAL_TICKS_PER_REV_RIGHT;
    b2_status_t st;
    float dl;
    float dr;
    float ds;
    float heading;
    float mid;

    b2_get_status(&st);
    if ((st.left_encoder_edges == s_pose.last_l) &&
        (st.right_encoder_edges == s_pose.last_r))
    {
        return;
    }
    dl = (float)s_bp_dir_l * (float)(st.left_encoder_edges - s_pose.last_l) * mm_l;
    dr = (float)s_bp_dir_r * (float)(st.right_encoder_edges - s_pose.last_r) * mm_r;
    s_pose.last_l = st.left_encoder_edges;
    s_pose.last_r = st.right_encoder_edges;
    s_pose.dist_l_mm += dl;
    s_pose.dist_r_mm += dr;
    s_pose.path_mm += 0.5f * (b4_fabsf(dl) + b4_fabsf(dr));
    ds = 0.5f * (dl + dr);
    heading = subsystem5_clampf((s_pose.dist_r_mm - s_pose.dist_l_mm) /
                                (float)B2_CAL_WHEEL_TRACK_MM, -B4M_PI, B4M_PI);
    mid = 0.5f * (s_pose.heading_rad + heading);
    s_pose.x_mm += ds * subsystem5_cos_rad(mid);
    s_pose.y_mm += ds * b4_sinf(mid);
    s_pose.heading_rad = heading;
}

/* Sideways position of the line sensors (they are ahead of the axle). */
static float subsystem5_sensor_y_mm(void)
{
    return s_pose.y_mm +
           ((float)subsystem5_SENSOR_TO_AXLE_MM * b4_sinf(s_pose.heading_rad));
}

/* Forward wheel requests, at most every BYPASS_UPDATE_MS. Both stay at or
 * above 0 (never a reversed wheel). Each moves by at most BYPASS_SLEW_MM_S
 * per update, so S2's speed loop keeps its integral. */
static void subsystem5_wheels(int32_t left, int32_t right, uint32_t now_us)
{
    const int32_t slew = (int32_t)subsystem5_BYPASS_SLEW_MM_S;

    if ((int32_t)(now_us - s_bp_next_cmd_us) < 0)
    {
        return;
    }
    s_bp_next_cmd_us = now_us + (subsystem5_BYPASS_UPDATE_MS * 1000U);
    left = subsystem5_clamp_i((left < 0) ? 0 : left,
                              s_bp_cmd_l - slew, s_bp_cmd_l + slew);
    right = subsystem5_clamp_i((right < 0) ? 0 : right,
                               s_bp_cmd_r - slew, s_bp_cmd_r + slew);
    subsystem5_drive(left, right);
}

/* Steer forward toward a heading at centre speed v: the wheel speed
 * difference grows with the heading error. */
static void subsystem5_steer(float goal_rad, int32_t v, uint32_t now_us)
{
    const float limit = (float)subsystem5_BYPASS_MAX_HEADING_DEG * S5_RAD_PER_DEG;
    const float error_deg =
        (subsystem5_clampf(goal_rad, -limit, limit) - s_pose.heading_rad) *
        S5_DEG_PER_RAD;
    const int32_t most = (v * (int32_t)subsystem5_BYPASS_STEER_DIFF_PCT) / 100;
    const int32_t diff = subsystem5_clamp_i(
        (int32_t)(error_deg * (float)subsystem5_BYPASS_STEER_GAIN), -most, most);

    subsystem5_wheels(v - diff, v + diff, now_us);   /* + error: turn left */
}

/* Heading that points at a spot `ahead_mm` further on, at sideways
 * position `lane_mm`. */
static float subsystem5_aim(float lane_mm, float ahead_mm)
{
    return b4_atan2f(lane_mm - s_pose.y_mm,
                     (ahead_mm > 1.0f) ? ahead_mm : 1.0f);
}

/* Sideways position (+ = left) of a ray reading; servo 90 = straight on,
 * angles above 90 to the left. */
static float subsystem5_ray_y_mm(uint16_t mm, float angle_deg)
{
    return -(float)mm * subsystem5_cosd(angle_deg);
}

/* Lane = edge + half the car + margin, at least that far from the line,
   at most BYPASS_OUT_MAX_MM. */
static void subsystem5_set_lane(void)
{
    const float clear = (float)(subsystem5_CAR_HALF_WIDTH_MM +
                                subsystem5_BYPASS_MARGIN_MM);

    s_bp_lane_mm = subsystem5_bp_side() *
        subsystem5_clampf(s_bp_edge_mm + clear, clear,
                          (float)subsystem5_BYPASS_OUT_MAX_MM);
}

/* 20 Nov: every reading during the bypass is a point on a small map, placed
 * with the car's pose and the servo angle. A point near the obstacle (from
 * its face to BYPASS_PASS_BLIND_MM behind it) and on the obstacle's side of
 * the car moves the obstacle's outer edge (-> the lane) and its back (->
 * where to turn back) out, by at most BYPASS_SIDE_STEP_MM per reading, so a
 * stray echo cannot swing the car. Walls and people further off are not on
 * the map. */
static void subsystem5_map_point(uint16_t mm, uint8_t servo_deg)
{
    const float s = subsystem5_bp_side();
    const float a = (float)subsystem5_SENSOR_TO_AXLE_MM;
    const float ang = s_pose.heading_rad + (((float)servo_deg - 90.0f) * S5_RAD_PER_DEG);
    const float px = s_pose.x_mm + (a * subsystem5_cos_rad(s_pose.heading_rad)) +
                     ((float)mm * subsystem5_cos_rad(ang));
    const float py = s_pose.y_mm + (a * b4_sinf(s_pose.heading_rad)) +
                     ((float)mm * b4_sinf(ang));
    const float face = s_bp_face_mm + a;
    const float step = (float)subsystem5_BYPASS_SIDE_STEP_MM;

    if ((px < (face - (float)subsystem5_FACE_TOL_MM)) ||
        (px > (face + (float)subsystem5_BYPASS_PASS_BLIND_MM)) ||
        ((s * py) >= (s * s_pose.y_mm)))
    {
        return;                       /* not the obstacle */
    }
    s_poll.bp_seen = 1U;
    if ((s * py) > s_bp_edge_mm)
    {
        s_bp_edge_mm = subsystem5_clampf(s * py, s_bp_edge_mm, s_bp_edge_mm + step);
        subsystem5_set_lane();
    }
    /* 20 Nov 15:08: an echo 25 cm ahead, beyond the box's far side, pushed
       the back on and the pass grew from 24 to 40 cm. The obstacle's back is
       on its side face, the one beside the car: only points within FACE_TOL
       of the edge move it. */
    if ((px > s_bp_back_mm) &&
        ((s * py) >= (s_bp_edge_mm - (float)subsystem5_FACE_TOL_MM)))
    {
        s_bp_back_mm = subsystem5_clampf(px, s_bp_back_mm, s_bp_back_mm + step);
    }
}

/* The servo sweeps this fan on the obstacle's side for the whole bypass
   (mirrored for a right pass): straight on, 30 deg and 60 deg off. */
static const uint8_t s_bp_fan[] = { 90U, 60U, 30U, 60U };

static void subsystem5_fan_next(void)
{
    const uint8_t n = (uint8_t)(sizeof(s_bp_fan) / sizeof(s_bp_fan[0]));
    uint8_t deg;

    s_bp_fan_i = (uint8_t)((s_bp_fan_i + 1U) % n);
    deg = s_bp_fan[s_bp_fan_i];
    subsystem5_servo_set((s_poll.bp_left != 0U) ? deg : (uint8_t)(180U - deg));
}

/* Both wheels measured (not just asked) below BYPASS_STOPPED_MM_S. */
static bool subsystem5_wheels_stopped(void)
{
    b2_status_t st;
    int32_t l;
    int32_t r;

    b2_get_status(&st);
    l = b2_speed_value(st.left_speed_ob);
    r = b2_speed_value(st.right_speed_ob);
    return (((l < 0) ? -l : l) < (int32_t)subsystem5_BYPASS_STOPPED_MM_S) &&
           (((r < 0) ? -r : r) < (int32_t)subsystem5_BYPASS_STOPPED_MM_S);
}

/* Lane and face from the profile, in the pose frame. */
static void subsystem5_plan_lanes(void)
{
    const subsystem5_profile_t *p = &s_status.profile;
    const float s = subsystem5_bp_side();
    const float clear = (float)(subsystem5_CAR_HALF_WIDTH_MM +
                                subsystem5_BYPASS_MARGIN_MM);
    float edge_mm;

    if ((p->right_edge_mm != 0U) && (p->left_edge_mm != 0U))
    {
        /* The passing-side edge, moved inward by the beam's half-width. */
        edge_mm = (s > 0.0f) ?
            subsystem5_ray_y_mm((uint16_t)p->left_edge_mm,
                (float)p->left_edge_deg - (float)subsystem5_BEAM_EDGE_DEG) :
            subsystem5_ray_y_mm((uint16_t)p->right_edge_mm,
                (float)p->right_edge_deg + (float)subsystem5_BEAM_EDGE_DEG);
    }
    else
    {
        edge_mm = subsystem5_ray_y_mm((uint16_t)p->closest_mm,
                                      (float)p->closest_angle_deg) +
                  (s * 0.5f * (float)p->approximate_width_mm);
    }

    if (s_bp_probe != 0U)
    {
        edge_mm = 0.0f;              /* unknown: OUT measures it */
    }
    s_bp_probe = 0U;
    s_bp_edge_mm = s * edge_mm;
    subsystem5_set_lane();
    /* Forward distance from the sensor to the face: the axle may travel this
       far before the front of the car reaches it. */
    s_bp_face_mm = (float)p->closest_mm *
                   b4_sinf((float)p->closest_angle_deg * S5_RAD_PER_DEG);
    if (s_bp_face_mm < 0.0f) { s_bp_face_mm = 0.0f; }
    /* Back: at least the course obstacle's measured depth; the sweep moves
       it further if it sees more. */
    s_bp_back_mm = s_bp_face_mm + (float)subsystem5_SENSOR_TO_AXLE_MM +
                   (float)subsystem5_OBSTACLE_DEPTH_MM;
}

static void subsystem5_bp_enter(uint8_t phase)
{
    s_poll.bp_phase = phase;
    s_status.bypass_phase = (uint8_t)(phase + 1U);   /* telemetry: 1..5 */
    s_bp_phase_path_mm = s_pose.path_mm;
    s_bp_clear_count = 0U;
    subsystem5_abort_pending_echo();
    /* Straight on to start each step; OUT/PASS/RETURN then sweep the fan. */
    s_bp_fan_i = 0U;
    subsystem5_servo_set(90U);
    if (phase == BP_TURN)
    {
        s_bp_turn_stopping = 0U;
    }
    if (phase == BP_RETURN)
    {
        /* At the approach angle the line sensors should reach the line
           after |y - line| / sin(angle); allow one sensor distance more. */
        s_bp_expect_mm = (b4_fabsf(s_pose.y_mm - s_bp_line_y_mm) /
                          b4_sinf((float)subsystem5_BYPASS_APPROACH_DEG *
                                  S5_RAD_PER_DEG)) +
                         (float)subsystem5_SENSOR_TO_AXLE_MM;
    }
    if (phase == BP_LOOK)
    {
        s_bp_looks++;
        s_bp_look_step = 0U;
        subsystem5_drive(0, 0);
    }
    if (phase == BP_ALIGN)
    {
        s_bp_al_step = 0U;
        s_bp_al_turns = 0U;
        s_bp_al_heading = s_pose.heading_rad;
        s_bp_al_start_mm = s_pose.path_mm;
    }
    tm_printf((UB*)"[S5] bypass: %s at x %d y %d mm, heading %d deg, line y %d\n",
              s_bp_name[phase], (int)s_pose.x_mm, (int)s_pose.y_mm,
              (int)(s_pose.heading_rad * S5_DEG_PER_RAD), (int)s_bp_line_y_mm);
}

static void subsystem5_bypass_fail(void)
{
    b2_stop();
    subsystem5_abort_pending_echo();
    subsystem5_fault(subsystem5_FAULT_BYPASS);
    tm_printf((UB*)"[S5] bypass failed during \"%s\" at x %d y %d mm, stopped\n",
              s_bp_name[s_poll.bp_phase], (int)s_pose.x_mm, (int)s_pose.y_mm);
}

static void subsystem5_bypass_begin(void)
{
    s_poll.bp_left = (s_status.profile.candidate_side == subsystem5_SIDE_LEFT) ?
                     1U : 0U;
    s_bp_started_us = subsystem5_now_us();
    s_bp_next_cmd_us = s_bp_started_us;
    s_bp_cmd_l = 0;
    s_bp_cmd_r = 0;
    s_bp_line_y_mm = 0.0f;
    s_bp_looks = 0U;
    s_bp_seen_path_mm = 0.0f;
    s_bp_prev_centre = 0U;
    subsystem5_pose_reset();
    s_bp_prev_sensor_y_mm = subsystem5_sensor_y_mm();
    subsystem5_plan_lanes();
    s_status.state = subsystem5_STATE_BYPASS;
    s_status.stop_required = 0U;  /* This motion is authorized: VC must not stop it. */
    tm_printf((UB*)"[S5] bypassing on the %s: lane %d mm, face %d mm ahead\n",
              (s_poll.bp_left != 0U) ? "left" : "right",
              (int)s_bp_lane_mm, (int)s_bp_face_mm);
    s_poll.bp_seen = 0U;
    subsystem5_bp_enter(BP_TURN);
}

/* Line sensors: remember where the line is.
 *  - The centre sensor reaching the tape: the line's middle is half a tape
 *    width further on, in the direction the sensor was moving.
 *  - Only the left sensor on it: the line is to the left of the centre
 *    sensor by an unmeasured amount, so keep moving the estimate left (at
 *    twice the align speed) until the centre sensor finds it. */
static LineBarcodeData_t subsystem5_watch_line(void)
{
    LineBarcodeData_t line = get_line_barcode_data_safe();
    const float sensor_y = subsystem5_sensor_y_mm();
    const float half = 0.5f * (float)subsystem5_LINE_WIDTH_MM;
    const uint8_t phase = s_poll.bp_phase;
    /* 19 Nov 22:17: black seen on the way round (a barcode stripe, another
     * part of the track) must not count as the line. Trusted: leaving the
     * original line at the start, or, from RETURN on, black near where the
     * line is expected. The window grows with the distance driven, because
     * the encoder pose drifts (wheel track error). */
    const float gate = (float)subsystem5_BYPASS_LINE_GATE_MM +
                       (((float)subsystem5_BYPASS_LINE_GATE_PCT / 100.0f) *
                        s_pose.path_mm);
    /* 20 Nov 15:08: black seen right after the turn on the spot (5 cm off
       the line) was trusted and moved the line estimate next to the car, so
       RETURN gave up after 6 cm. The car starts with the centre sensor on
       the line (y = 0): only trust sightings from RETURN on. */
    const bool trusted =
        (((phase == BP_RETURN) || (phase == BP_ALIGN) || (phase == BP_LOOK)) &&
         (b4_fabsf(sensor_y - s_bp_line_y_mm) <= gate));

    if (!trusted)
    {
        line.line_detected = 0U;     /* not the line we are looking for */
        line.ir_left_raw = 0U;
        line.ir_center_raw = 0U;
    }
    if (line.line_detected != 0U)
    {
        s_bp_seen_path_mm = s_pose.path_mm;
    }
    if ((line.ir_center_raw != 0U) && (s_bp_prev_centre == 0U))
    {
        s_bp_line_y_mm = sensor_y +
            ((sensor_y > s_bp_prev_sensor_y_mm) ? half :
             ((sensor_y < s_bp_prev_sensor_y_mm) ? -half : 0.0f));
    }
    else if ((line.ir_left_raw != 0U) && (line.ir_center_raw == 0U))
    {
        /* Only the left sensor on it: the line is LEFT_IR_OFFSET_MM to the
           left of the centre sensor (measured on the car). */
        s_bp_line_y_mm = sensor_y + ((float)subsystem5_LEFT_IR_OFFSET_MM *
                                     subsystem5_cos_rad(s_pose.heading_rad));
    }
    else
    {
        /* nothing new */
    }
    s_bp_prev_centre = line.ir_center_raw;
    s_bp_prev_sensor_y_mm = sensor_y;
    return line;
}

/* Hand the car back to S3 WITHOUT stopping it (20 Nov): S3 picks it up on
   its next loop, steering by the line. S3 starts following at once after a
   bypass, and searches for the line if a sensor is just off it. */
static void subsystem5_bypass_done(uint32_t now_us, const char *how)
{
    tm_printf((UB*)"[S5] bypass done in %u ms, %s "
              "(y %d mm, heading %d deg): S3 follows\n",
              (unsigned)(subsystem5_elapsed(s_bp_started_us, now_us) / 1000U),
              how, (int)s_pose.y_mm, (int)(s_pose.heading_rad * S5_DEG_PER_RAD));
    subsystem5_reset_for_monitor();
}

static void subsystem5_bypass_step(uint32_t now_us)
{
    const float s = subsystem5_bp_side();
    const float lookahead = (float)subsystem5_BYPASS_LOOKAHEAD_MM;
    const float a = (float)subsystem5_SENSOR_TO_AXLE_MM;
    const float toward = -s;             /* sideways direction to the line */
    LineBarcodeData_t line;
    subsystem5_sample_t r;
    bool have = false;
    b2_status_t st;
    float travelled;

    b2_get_status(&st);
    if (!s1_mission_go())
    {
        b2_stop();
        tm_printf((UB*)"[S5] STOP during the bypass: cancelled\n");
        subsystem5_reset_for_monitor();
        return;
    }
    if ((st.brake_latched != 0U) || (st.state == B2_FAULT) ||
        (subsystem5_elapsed(s_bp_started_us, now_us) >
         (subsystem5_BYPASS_TIMEOUT_MS * 1000U)))
    {
        subsystem5_bypass_fail();
        return;
    }

    subsystem5_pose_update();
    travelled = s_pose.path_mm - s_bp_phase_path_mm;
    line = subsystem5_watch_line();

    if ((s_poll.range_active == 0U) &&
        ((int32_t)(now_us - s_servo_ready_us) >= 0))
    {
        (void)subsystem5_ping_begin(now_us);
    }
    r.angle_deg = s_servo_angle;
    if (subsystem5_ping_result(now_us, &r))
    {
        const uint8_t looked = s_servo_angle;

        if (r.valid != 0U)
        {
            have = true;
            if (looked == 90U)
            {
                s_status.front_valid = (r.distance_mm != subsystem5_FAR_MM) ? 1U : 0U;
                s_status.front_mm = r.distance_mm;
            }
            if (r.distance_mm <= subsystem5_OBSTACLE_ZONE_MM)
            {
                subsystem5_map_point(r.distance_mm, looked);
            }
        }
        /* Keep sweeping while driving round; straight on while turning on
           the spot and while aligning (the front check needs it). */
        if ((s_poll.bp_phase == BP_OUT) || (s_poll.bp_phase == BP_PASS) ||
            (s_poll.bp_phase == BP_RETURN))
        {
            subsystem5_fan_next();
        }
    }

    /* Driving forward, looking ahead: never into something. */
    if (have && (s_servo_angle == 90U) && (r.distance_mm <= subsystem5_URGENT_MM) &&
        ((s_poll.bp_phase == BP_OUT) || (s_poll.bp_phase == BP_RETURN) ||
         (s_poll.bp_phase == BP_ALIGN)))
    {
        subsystem5_bypass_fail();
        return;
    }

    switch (s_poll.bp_phase)
    {
    case BP_TURN:
    {
        /* Turn on the spot toward the passing side, sensor straight on,
           until it looks past the obstacle's edge AND the car points at the
           lane (edge + half the car + margin, level with the face). Each
           wheel gets a firmer request until it is measured turning (a wheel
           started backwards at 80 mm/s stalled S2 on 19 Nov). */
        const float need = b4_atan2f(s * s_bp_lane_mm,
                                     s_bp_face_mm + (float)subsystem5_SENSOR_TO_AXLE_MM);

        if (have && (s_servo_angle == 90U))
        {
            if (r.distance_mm <= (uint16_t)(s_bp_face_mm + (float)subsystem5_FACE_TOL_MM))
            {
                s_bp_clear_count = 0U;
            }
            else if (s_bp_clear_count < UINT8_MAX)
            {
                s_bp_clear_count++;
            }
            else
            {
                /* saturated */
            }
        }
        if (s_bp_turn_stopping != 0U)
        {
            if (subsystem5_wheels_stopped())
            {
                s_bp_cmd_l = 0;
                s_bp_cmd_r = 0;
                subsystem5_bp_enter(BP_OUT);
            }
        }
        else if ((s_bp_clear_count >= 2U) && ((s * s_pose.heading_rad) >= need))
        {
            subsystem5_drive(0, 0);      /* stop before driving forward */
            s_bp_turn_stopping = 1U;
        }
        else if ((s * s_pose.heading_rad) >
                 ((float)subsystem5_BYPASS_TURN_MAX_DEG * S5_RAD_PER_DEG))
        {
            subsystem5_bypass_fail();    /* never saw past it */
        }
        else
        {
            const int32_t v = (int32_t)subsystem5_BYPASS_TURN_SPEED_MM_S;
            const int32_t kick = (int32_t)subsystem5_REVERSE_START_MM_S;
            const int32_t ls = b2_speed_value(st.left_speed_ob);
            const int32_t rs = b2_speed_value(st.right_speed_ob);
            int32_t lw;
            int32_t rw;

            lw =((((ls < 0) ? -ls : ls) < (int32_t)subsystem5_BYPASS_STOPPED_MM_S) ? kick : v);
            rw = ((((rs < 0) ? -rs : rs) < (int32_t)subsystem5_BYPASS_STOPPED_MM_S) ? kick : v);
            /* Left pass: left wheel back, right wheel forward. */
            subsystem5_drive((s > 0.0f) ? -lw : lw, (s > 0.0f) ? rw : -rw);
        }
        break;
    }

    case BP_OUT:
        /* Aim at the lane where the front meets the face; once that point
           is close, aim a look-ahead down the lane instead. */
        subsystem5_steer(subsystem5_aim(s_bp_lane_mm,
            ((s_bp_face_mm - s_pose.x_mm) > (0.5f * lookahead)) ?
                (s_bp_face_mm - s_pose.x_mm) : (0.5f * lookahead)),
            (int32_t)subsystem5_BYPASS_SPEED_MM_S, now_us);
        if ((s * s_pose.y_mm) >=
            ((s * s_bp_lane_mm) - (float)subsystem5_BYPASS_LANE_TOL_MM))
        {
            subsystem5_bp_enter(BP_PASS);
        }
        else if (travelled > (float)subsystem5_BYPASS_OUT_MAX_MM)
        {
            subsystem5_bypass_fail();    /* never reached the lane */
        }
        else
        {
            /* still moving out */
        }
        break;

    case BP_PASS:
        /* Hold the lane while the sweep maps the obstacle's side and back;
           turn back once the axle is BYPASS_PASS_AFTER_MM past its back. */
        subsystem5_steer(subsystem5_aim(s_bp_lane_mm, lookahead),
                         (int32_t)subsystem5_BYPASS_SPEED_MM_S, now_us);
        if (s_pose.x_mm >= (s_bp_back_mm + (float)subsystem5_BYPASS_PASS_AFTER_MM))
        {
            subsystem5_bp_enter(BP_RETURN);
        }
        else if (travelled > (float)subsystem5_BYPASS_PASS_MAX_MM)
        {
            subsystem5_bypass_fail();    /* still beside it: too long */
        }
        else
        {
            /* still passing */
        }
        break;

    case BP_RETURN:
    {
        /* 20 Nov: close on the line at a steady angle until a sensor finds
           it: the centre-sensor line-up works best from a clean, known
           crossing (simulation: 60/108 vs 53 with the old aimed approach). */
        /* 20 Nov: back to the SAME line it was following, just behind the
           obstacle: aim at the line one sensor distance past the mapped
           back, so the car turns in as soon as it is past (steeper when the
           back is close), never shallower than BYPASS_APPROACH_DEG. */
        const float ahead = (s_bp_back_mm + a) - s_pose.x_mm;
        float goal = subsystem5_aim(s_bp_line_y_mm, (ahead > a) ? ahead : a);

        if ((toward * goal) <
            ((float)subsystem5_BYPASS_APPROACH_DEG * S5_RAD_PER_DEG))
        {
            goal = toward * (float)subsystem5_BYPASS_APPROACH_DEG * S5_RAD_PER_DEG;
        }

        /* Never turn in toward the mapped back corner closer than half the
           car + the margin, while it is still ahead of the axle (12:30: kept
           until 12 cm behind, it levelled the car off beside the line). */
        {
            const float dx = s_bp_back_mm - s_pose.x_mm;
            const float dy = (s * s_bp_edge_mm) - s_pose.y_mm;
            const float rho = b4_sqrtf((dx * dx) + (dy * dy));
            const float clear = (float)(subsystem5_CAR_HALF_WIDTH_MM +
                                        subsystem5_BYPASS_MARGIN_MM);
            const float ahead = (dx * subsystem5_cos_rad(s_pose.heading_rad)) +
                                (dy * b4_sinf(s_pose.heading_rad));

            if ((rho > clear) && (ahead > 0.0f))
            {
                const float lim = b4_atan2f(dy, dx) +
                    (s * b4_atan2f(clear, b4_sqrtf((rho * rho) - (clear * clear))));

                if ((s * goal) < (s * lim))
                {
                    goal = lim;
                }
            }
        }
        /* Slow down near where the line should be: less to overshoot. */
        subsystem5_steer(goal,
            (b4_fabsf(s_pose.y_mm - s_bp_line_y_mm) < (2.0f * a)) ?
                (int32_t)subsystem5_BYPASS_ALIGN_SPEED_MM_S :
                (int32_t)subsystem5_BYPASS_SPEED_MM_S, now_us);
        if (line.line_detected != 0U)
        {
            subsystem5_bp_enter(BP_ALIGN);
        }
        else if (((toward * (subsystem5_sensor_y_mm() - s_bp_line_y_mm)) >
                  ((float)subsystem5_BYPASS_LINE_GATE_MM +
                   (((float)subsystem5_BYPASS_LINE_GATE_PCT / 100.0f) * s_pose.path_mm))) ||
                 (travelled > (2.0f * s_bp_expect_mm)))
        {
            /* 20 Nov: the sensors are past where the line can be (the
               same window watch_line trusts) without seeing it: stop and look
               both ways. 15:08: a fixed travel stopped it 6 cm into the
               return, short of the line. */
            subsystem5_bp_enter(BP_LOOK);
        }
        else
        {
            /* still returning */
        }
        break;

    case BP_LOOK:
        /* Turn on the spot toward the line's side first, up to
           BYPASS_LOOK_DEG, then the other way to BYPASS_LOOK_DEG past the
           start: the line sensors, SENSOR_TO_AXLE ahead of the axle, sweep
           an arc across the floor. Any sensor on the line: line up. */
        if (line.line_detected != 0U)
        {
            s_bp_cmd_l = 0;
            s_bp_cmd_r = 0;
            subsystem5_drive(0, 0);
            subsystem5_bp_enter(BP_ALIGN);
        }
        else if (s_bp_look_step == 0U)
        {
            if (subsystem5_wheels_stopped())
            {
                s_bp_look_h0 = s_pose.heading_rad;
                s_bp_look_dir = ((s_bp_line_y_mm - subsystem5_sensor_y_mm()) >= 0.0f) ?
                                1 : -1;          /* + = the line is to the left */
                s_bp_look_step = 1U;
            }
        }
        else if (((float)s_bp_look_dir * (s_pose.heading_rad - s_bp_look_h0)) >=
                 ((float)subsystem5_BYPASS_LOOK_DEG * S5_RAD_PER_DEG))
        {
            if (s_bp_look_step == 1U)
            {
                s_bp_look_dir = (int8_t)(-s_bp_look_dir);
                s_bp_look_step = 2U;
            }
            else
            {
                subsystem5_drive(0, 0);
                subsystem5_bypass_done(now_us, "looked both ways, S3 searches");
            }
        }
        else
        {
            const int32_t v = (int32_t)subsystem5_BYPASS_TURN_SPEED_MM_S;
            const int32_t kick = (int32_t)subsystem5_REVERSE_START_MM_S;
            const int32_t ls = b2_speed_value(st.left_speed_ob);
            const int32_t rs = b2_speed_value(st.right_speed_ob);
            const int32_t lw = ((((ls < 0) ? -ls : ls) <
                                 (int32_t)subsystem5_BYPASS_STOPPED_MM_S) ? kick : v);
            const int32_t rw = ((((rs < 0) ? -rs : rs) <
                                 (int32_t)subsystem5_BYPASS_STOPPED_MM_S) ? kick : v);

            /* +1: turn left (left wheel back), -1: turn right. */
            subsystem5_drive((s_bp_look_dir > 0) ? -lw : lw,
                             (s_bp_look_dir > 0) ? rw : -rw);
        }
        break;
    }

    case BP_ALIGN:
    default:
        /* 20 Nov 14:46: straightening up as soon as the centre sensor
           touched the tape left the car beside the line (the wheels were
           still ~7 cm off it): the sensor slid off and the line-up took 20 s.
           Now: the line sensors are SENSOR_TO_AXLE_MM ahead of the axle, so
           drive on that far along the same heading -> the axle is where the
           sensor crossed the tape. Then turn on the spot back toward the
           line's direction: the centre sensor, swinging round the axle,
           comes back over the tape when the car is parallel to it. Stop
           there; if the wheels coasted past, turn back the other way.
           Simulation (23x12 box, wheels +-10 %, lag 0.1-0.4 s): centre on
           the tape 90/90, heading within ~8 deg (S3 finishes it). */
        if (s_bp_al_step == 0U)
        {
            if ((s_pose.path_mm - s_bp_al_start_mm) >= a)
            {
                subsystem5_drive(0, 0);
                s_bp_cmd_l = 0;
                s_bp_cmd_r = 0;
                s_bp_al_step = 1U;
            }
            else
            {
                subsystem5_steer(s_bp_al_heading,
                                 (int32_t)subsystem5_BYPASS_ALIGN_SPEED_MM_S,
                                 now_us);
            }
        }
        else if ((s_bp_al_step == 1U) || (s_bp_al_step == 3U))
        {
            if (subsystem5_wheels_stopped())
            {
                if ((s_bp_al_step == 3U) && (line.ir_center_raw != 0U))
                {
                    subsystem5_bypass_done(now_us,
                        "parallel, centre sensor over the line");
                }
                else if (s_bp_al_turns >= subsystem5_BYPASS_ALIGN_TURNS)
                {
                    if (s_bp_looks < subsystem5_BYPASS_MAX_LOOKS)
                    {
                        subsystem5_bp_enter(BP_LOOK);
                    }
                    else
                    {
                        subsystem5_bypass_done(now_us, "S3 finishes the alignment");
                    }
                }
                else
                {
                    if (s_bp_al_step == 1U)
                    {
                        /* back toward the line's direction (heading 0) */
                        s_bp_al_dir = (s_pose.heading_rad > 0.0f) ? -1 : 1;
                    }
                    else
                    {
                        s_bp_al_dir = (int8_t)(-s_bp_al_dir);   /* coasted past */
                    }
                    s_bp_al_turn_h0 = s_pose.heading_rad;
                    s_bp_al_turns++;
                    s_bp_al_step = 2U;
                }
            }
        }
        else
        {
            if ((line.ir_center_raw != 0U) ||
                (((float)s_bp_al_dir * (s_pose.heading_rad - s_bp_al_turn_h0)) >
                 ((float)subsystem5_BYPASS_LOOK_DEG * S5_RAD_PER_DEG)))
            {
                subsystem5_drive(0, 0);
                s_bp_cmd_l = 0;
                s_bp_cmd_r = 0;
                s_bp_al_step = 3U;
            }
            else
            {
                const int32_t v = (int32_t)subsystem5_BYPASS_TURN_SPEED_MM_S;
                const int32_t kick = (int32_t)subsystem5_REVERSE_START_MM_S;
                const int32_t ls = b2_speed_value(st.left_speed_ob);
                const int32_t rs = b2_speed_value(st.right_speed_ob);
                const int32_t lw = ((((ls < 0) ? -ls : ls) <
                                     (int32_t)subsystem5_BYPASS_STOPPED_MM_S) ? kick : v);
                const int32_t rw = ((((rs < 0) ? -rs : rs) <
                                     (int32_t)subsystem5_BYPASS_STOPPED_MM_S) ? kick : v);

                /* +1: turn left (left wheel back), -1: turn right. */
                subsystem5_drive((s_bp_al_dir > 0) ? -lw : lw,
                                 (s_bp_al_dir > 0) ? rw : -rw);
            }
        }
        break;
    }
}

/* No safe side: back off a little and scan again (briefing: "plan stop,
 * turn or reverse actions"). The motion is authorized, like the bypass. */
static void subsystem5_reverse_begin(void)
{
    b2_command_t cmd;

    subsystem5_memset(&cmd, 0, (int)sizeof(cmd));
    cmd.type = B2_CMD_MOVE_BACKWARD;
    cmd.distance_mm = subsystem5_REVERSE_MM;
    cmd.goal_speed_mm_s = subsystem5_REVERSE_START_MM_S;  /* see ultrasonic.h */
    cmd.timeout_ms = 3000U;
    subsystem5_abort_pending_echo();
    subsystem5_servo_set(90U);
    tm_printf((UB*)"[S5] no safe side: reversing %u mm to rescan\n",
              (unsigned)subsystem5_REVERSE_MM);
    if (!b2_goal_start(&s_bypass_goal, &cmd))
    {
        subsystem5_fault(subsystem5_FAULT_NO_SAFE_SIDE);
        return;
    }
    s_status.stop_required = 0U;      /* VC must not stop this motion */
    s_status.state = subsystem5_STATE_REVERSE;
}

static void subsystem5_reverse_step(void)
{
    const b2_goal_result_t result = b2_goal_check(&s_bypass_goal);

    if (!s1_mission_go())
    {
        b2_stop();
        tm_printf((UB*)"[S5] STOP while reversing: cancelled\n");
        subsystem5_reset_for_monitor();
    }
    else if (result == B2_GOAL_DONE)
    {
        s_status.stop_required = 1U;  /* still until the rescan is done */
        s_scan_retries = 0U;
        subsystem5_scan_start();
    }
    else if (result == B2_GOAL_FAILED)
    {
        b2_stop();
        subsystem5_fault(subsystem5_FAULT_NO_SAFE_SIDE);
    }
    else
    {
        /* still reversing */
    }
}
#endif /* APP_MISSION_ENABLE */

void subsystem5_poll(void)
{
    const uint32_t now_us = subsystem5_now_us();
    subsystem5_sample_t front;

    if ((s_line_recovered_ack != 0U) &&
        (s_status.state == subsystem5_STATE_PROFILE_READY) &&
        (s_status.scan_complete != 0U))
    {
        s_line_recovered_ack = 0U;
        subsystem5_reset_for_monitor();
    }
    else if ((s_fault_reset_request != 0U) &&
             (s_status.state == subsystem5_STATE_FAULT))
    {
        s_fault_reset_request = 0U;
        subsystem5_reset_for_monitor();
    }
    else
    {
        s_line_recovered_ack = 0U;
        s_fault_reset_request = 0U;
    }

    if (s_status.state == subsystem5_STATE_MONITOR)
    {
        if (s_stationary_scan_requested != 0U)
        {
            s_stationary_scan_requested = 0U;
            subsystem5_abort_pending_echo();
            s_status.stop_required = 1U;
            s_stop_request_us = now_us;
            s_scan_retries = 0U;
            s_reverse_count = 0U;
            s_status.state = subsystem5_STATE_WAIT_STOP;
            subsystem5_send_event(subsystem5_EVENT_STOP_REQUIRED, subsystem5_FAULT_NONE);
            subsystem5_snapshot_update();
            return;
        }

        if ((s_poll.range_active == 0U) &&
            ((int32_t)(now_us - s_servo_ready_us) >= 0))
        {
            (void)subsystem5_ping_begin(now_us);
        }
        front.angle_deg = 90U;
        if (subsystem5_ping_result(now_us, &front))
        {
            const bool far = (front.valid != 0U) &&
                             (front.distance_mm == subsystem5_FAR_MM);

            /* Open space is reported as "no echo" to telemetry, as before. */
            s_status.front_valid = far ? 0U : front.valid;
            s_status.front_mm = far ? subsystem5_UNKNOWN_MM : front.distance_mm;
            if (front.valid == 0U)
            {
                s_close_count = 0U;
                if (s_monitor_invalid < subsystem5_MAX_INVALID_MONITOR)
                {
                    s_monitor_invalid++;
                }
                if (s_monitor_invalid >= subsystem5_MAX_INVALID_MONITOR)
                {
                    subsystem5_fault(subsystem5_FAULT_MONITOR_SENSOR);
                }
            }
            else
            {
                s_monitor_invalid = 0U;
                /* INTEGRATION FIX: only react while S3 is following the line.
                 * A parked car must not start a bypass, and S3's junction
                 * moves and turns must not be cut short. */
                if ((!subsystem5_watch_active()) ||
                    (front.distance_mm > subsystem5_DETECT_MM))
                {
                    s_close_count = 0U;
                }
                else
                {
                    if (s_close_count < UINT8_MAX)
                    {
                        s_close_count++;
                    }
                    /* INTEGRATION FIX: 2 close readings in a row, or 1 when
                     * under subsystem5_URGENT_MM, so one stray echo does not
                     * stop the car.  Still no wait for the slow servo scan. */
                    if ((front.distance_mm < subsystem5_URGENT_MM) ||
                        (s_close_count >= subsystem5_CLOSE_READINGS))
                    {
                        s_close_count = 0U;
                        s_status.stop_required = 1U;
                        s_stop_request_us = now_us;
                        s_scan_retries = 0U;
                        s_reverse_count = 0U;
                        s_status.state = subsystem5_STATE_WAIT_STOP;
                        subsystem5_send_event(subsystem5_EVENT_STOP_REQUIRED, subsystem5_FAULT_NONE);
                    }
                }
            }
        }
    }
    else if (s_status.state == subsystem5_STATE_WAIT_STOP)
    {
        if (s_stopped_confirmed != 0U)
        {
            s_stopped_confirmed = 0U;
            subsystem5_scan_start();
        }
        else if (subsystem5_elapsed(s_stop_request_us, now_us) >=
                 (subsystem5_STOP_TIMEOUT_MS * 1000U))
        {
            subsystem5_fault(subsystem5_FAULT_STOP_TIMEOUT);
        }
        else
        {
            /* Non-blocking wait for MotionControl confirmation. */
        }
    }
    else if ((s_status.state == subsystem5_STATE_COARSE) ||
             (s_status.state == subsystem5_STATE_FINE))
    {
        subsystem5_scan_step(now_us);
    }
#if APP_MISSION_ENABLE
    else if (s_status.state == subsystem5_STATE_PROFILE_READY)
    {
        /* INTEGRATION FIX: STOP (or E-STOP) during the scan clears
         * s1_mission_go(); then the bypass must not drive.  Drop the scan;
         * after START, S5 sees the obstacle again and rescans. */
        if (s1_mission_go())
        {
            subsystem5_bypass_begin();
        }
        else
        {
            tm_printf((UB*)"[S5] STOP during the scan: bypass cancelled\n");
            subsystem5_reset_for_monitor();
        }
    }
    else if (s_status.state == subsystem5_STATE_BYPASS)
    {
        subsystem5_bypass_step(now_us);
    }
    else if (s_status.state == subsystem5_STATE_REVERSE)
    {
        subsystem5_reverse_step();
    }
#endif
    else
    {
        /* PROFILE_READY / FAULT: remain stopped until VC authorizes. */
    }
    subsystem5_snapshot_update();
}

int subsystem5_ultrasonic_init(subsystem5_event_publish_fn publish, void *context)
{
    T_CMTX cmtx = {0};

    /* PORT: mutex that protects the status snapshot */
    cmtx.mtxatr = TA_INHERIT;
    s_mtx = tk_cre_mtx(&cmtx);
    if (s_mtx < E_OK)
    {
        return s_mtx;
    }

    subsystem5_memset(&s_status, 0, (int)sizeof(s_status));
    subsystem5_memset(&s_snapshot, 0, (int)sizeof(s_snapshot));
    subsystem5_memset(&s_samples, 0, (int)sizeof(s_samples));
    s_status.state = subsystem5_STATE_MONITOR;
    s_publish = publish;
    s_publish_context = context;
    s_poll.range_active = 0U;
    s_monitor_invalid = 0U;
    s_close_count = 0U;
    s_scan_retries = 0U;
    s_scan_index = 0U;
    s_stopped_confirmed = 0U;
    s_stationary_scan_requested = 0U;
    s_line_recovered_ack = 0U;
    s_fault_reset_request = 0U;
    s_servo_angle = 90U;
    s_last_ping_us = subsystem5_now_us() - subsystem5_PING_GAP_US;

    /* PORT: gpio_init/gpio_set_dir/gpio_put -> libbsp.  The old
     * gpio_disable_pulls() has no libbsp equivalent; the HC-SR04 drives the
     * echo line itself so no pull is needed. */
    (void)gpio_set_pin(subsystem5_TRIG_GPIO, GPIO_MODE_OUT);
    (void)gpio_set_val(subsystem5_TRIG_GPIO, 0U);
    (void)gpio_set_pin(subsystem5_ECHO_GPIO, GPIO_MODE_IN);

    s_poll.echo_phase = 0U;
    s_echo_rise_us = 0U;
    s_echo_pulse_us = 0U;

    /* PORT: PWM through libbsp + one direct write for the clock divider.
     * INTEGRATION FIX: the kernel does not take the PWM block out of reset
     * (USE_PTMR = 0), so without this every PWM register write was ignored
     * and the servo never moved.  Release only -- never reset it, because
     * S2's motor PWM shares the block. */
    app_release_reset(RESETS_RESET_PWM);
    (void)pwm_set_pin(subsystem5_SERVO_GPIO);
    subsystem5_PWM_CH_DIV = subsystem5_PWM_DIV_1MHZ;
    (void)pwm_set_wrap(subsystem5_SERVO_GPIO, subsystem5_SERVO_PERIOD_US - 1U);
    (void)pwm_set_enabled(subsystem5_SERVO_GPIO, TRUE);
    subsystem5_servo_set(90U);
    subsystem5_snapshot_update();
    return E_OK;
}

void subsystem5_confirm_robot_stopped(void)
{
    s_stopped_confirmed = 1U;
}

bool subsystem5_request_stationary_scan(void)
{
    if (s_status.state != subsystem5_STATE_MONITOR)
    {
        return false;
    }
    s_stationary_scan_requested = 1U;
    return true;
}

void subsystem5_ack_line_recovered(void)
{
    /* Caller guarantees VC-approved bypass and successful line reacquisition.
     * Request is handled by Subsystem 5's task, not in the caller's task. */
    s_line_recovered_ack = 1U;
}

void subsystem5_reset_fault_while_stopped(void)
{
    /* Deliberate reset is handled in Subsystem 5's task. */
    s_fault_reset_request = 1U;
}

void subsystem5_get_status(subsystem5_status_t *result)
{
    if (result == NULL)
    {
        return;
    }
    /* PORT: mutex-protected copy (safe from any task on either core). */
    subsystem5_lock();
    *result = s_snapshot;
    subsystem5_unlock();
}


/* =====================================================================
 *  Subsystem 5 glue (called from app_program/app_main.c)
 *  -- merged in from subsystem5.c / the old tkernel adapter
 * ===================================================================== */

EXPORT ID	mbf_s5_events;	/* IF-02, see ultrasonic.h */

/* The single task that owns GP0/GP1 (ultrasonic) and GP12 (servo). */
static void subsystem5_task(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;

	/* 20 Nov: power-on servo check. Every scan since 19 Nov 19:39 read
	   the same at every angle (servo not turning). One visible sweep at
	   start-up, before anything drives: if the sensor does not swing left
	   and right now, check the servo's power and its signal wire on GP12. */
	tm_printf((UB*)"[S5] servo check: sweeping 30 -> 150 -> 90 deg\n");
	subsystem5_servo_set(subsystem5_SERVO_MIN_DEG);
	tk_dly_tsk(500);
	subsystem5_servo_set(subsystem5_SERVO_MAX_DEG);
	tk_dly_tsk(700);
	subsystem5_servo_set(90U);
	tk_dly_tsk(400);

	while (1) {
		subsystem5_poll();
		tk_dly_tsk(S5_POLL_PERIOD_MS);
	}
}

/* Publish callback required by the driver: bounded and non-blocking
   (TMO_POL), returns false when the buffer is full. */
static bool subsystem5_publish(const subsystem5_event_t *event, void *context)
{
	(void)context;
	return tk_snd_mbf(mbf_s5_events, event, (INT)sizeof(*event), TMO_POL) >= E_OK;
}

EXPORT ER subsystem5_init(void)
{
	T_CMBF	cmbf = {0};

	/* Message buffer for events: room for 8 queued events (+ headers) */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 8 * ((INT)sizeof(subsystem5_event_t) + 8);
	cmbf.maxmsz = (INT)sizeof(subsystem5_event_t);
	mbf_s5_events = tk_cre_mbf(&cmbf);
	if (mbf_s5_events < E_OK) {
		return mbf_s5_events;
	}

	/* Pins, PWM, mutex, state */
	return subsystem5_ultrasonic_init(subsystem5_publish, NULL);
}

EXPORT ER subsystem5_start(void)
{
	/* Core 0 (TP_PRC1): the subsystem 3 barcode task busy-polls on core 1 at
	   priority 16 and would starve this task (priority 20) if it were pinned
	   there. */
	ID tid = app_start_task(subsystem5_task, S5_PRI_MAIN, TP_PRC1,
				"ultrasonic_task");
	return (tid < E_OK) ? tid : E_OK;
}