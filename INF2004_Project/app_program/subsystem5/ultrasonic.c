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
    uint16_t bp_phase : 3;          /* BP_TURN_OUT..BP_ALIGN, 7 values */
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

static void subsystem5_prepare_fine(void)
{
    uint16_t closest_mm = UINT16_MAX;
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
        if ((point->valid != 0U) &&
            (point->distance_mm <= subsystem5_OBSTACLE_ZONE_MM) &&
            (point->distance_mm < closest_mm))
        {
            closest_mm = point->distance_mm;
            focus = (uint8_t)point->angle_deg;
        }
    }

    if ((closest_mm == UINT16_MAX) && (invalid == 0U))
    {
        /* INTEGRATION FIX: every ray answered and nothing is there (the
         * obstacle moved, or it was a stray echo): carry on, not a fault. */
        tm_printf((UB*)"[S5] scan found nothing, carrying on\n");
        subsystem5_reset_for_monitor();
        return;
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
            subsystem5_fault(subsystem5_FAULT_SCAN_INVALID);
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
            subsystem5_fault(subsystem5_FAULT_SCAN_INVALID);
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

    s_status.scan_complete = 1U;
    s_status.state = subsystem5_STATE_PROFILE_READY;
    subsystem5_servo_set(90U);
    subsystem5_send_event(subsystem5_EVENT_PROFILE_READY, subsystem5_FAULT_NONE);
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
/* Obstacle bypass, driven by the SENSORS rather than exact turn angles: the
 * car's pivot turns are off by tens of degrees (17 Nov: a "90 degree" turn
 * went ~105 by the encoders, ~150 by eye), so a fixed rectangle of 90-degree
 * turns drove off in the wrong direction.
 *   TURN_OUT   pivot away from the obstacle until the front ray sees past it,
 *              then a little more for the car body.
 *   OUT        straight, far enough sideways to clear the obstacle's edge.
 *   TURN_BACK  pivot back by the SAME encoder count as TURN_OUT. Both turns
 *              share any turn error, so the heading returns to the line's.
 *   PASS       straight, the sensor looking at the obstacle 60 degrees to the
 *              side, until the obstacle is behind the car.
 *   TURN_IN    pivot toward the line by the same count again.
 *   SEEK       straight until the line sensors find the line.
 *   ALIGN      pivot back by the same count (line direction), then line
 *              following takes over.
 * Speed commands + this task's own encoder counting, so every phase can end
 * on a sensor reading. */
#define BP_TURN_OUT   (0U)
#define BP_OUT        (1U)
#define BP_TURN_BACK  (2U)
#define BP_PASS       (3U)
#define BP_TURN_IN    (4U)
#define BP_SEEK       (5U)
#define BP_ALIGN      (6U)

static const char *const s_bp_name[] =
{
    "turn out", "out", "turn back", "pass", "turn in", "seek line", "align"
};

static b2_goal_t s_bypass_goal;      /* used by the reverse step */
static uint32_t s_bp_base_ticks;     /* encoder count at the phase start */
static uint32_t s_bp_turn_ticks;     /* encoder count of TURN_OUT */
static uint32_t s_bp_clear_at;       /* TURN_OUT: count when clear, 0 = not yet */
static uint8_t s_bp_clear_count;
static uint32_t s_bp_passed_at;      /* PASS: count when passed, 0 = not yet */
static uint32_t s_bp_out_mm;
static uint32_t s_bp_line_at;        /* SEEK: count when the line was found */
static uint32_t s_bp_started_us;

/* +1 pass on the right, -1 on the left (CR1: stored as 1 bit). */
static int32_t subsystem5_bp_dir(void)
{
    return (s_poll.bp_left != 0U) ? -1 : 1;
}

/* Mean of the two wheels' encoder edges (single-channel: always counts up). */
static uint32_t subsystem5_ticks_now(void)
{
    b2_status_t st;
    b2_get_status(&st);
    return (st.left_encoder_edges + st.right_encoder_edges) / 2U;
}

static uint32_t subsystem5_mm_to_ticks(uint32_t mm)
{
    return (mm * subsystem5_TICKS_PER_M) / 1000U;
}

static uint32_t subsystem5_deg_to_ticks(uint32_t deg)
{
    return (deg * subsystem5_TICKS_PER_DEG_X10) / 10U;
}

static void subsystem5_drive(int32_t left_mm_s, int32_t right_mm_s)
{
    b2_command_t cmd;

    subsystem5_memset(&cmd, 0, (int)sizeof(cmd));
    cmd.type = B2_CMD_SPEED;
    cmd.left_mm_s = (int16_t)left_mm_s;
    cmd.right_mm_s = (int16_t)right_mm_s;
    (void)b2_submit_command(&cmd);
}

static void subsystem5_bp_enter(uint8_t phase)
{
    const int32_t t = (int32_t)subsystem5_BYPASS_TURN_MM_S * subsystem5_bp_dir();
    const int32_t v = (int32_t)subsystem5_BYPASS_SPEED_MM_S;

    s_poll.bp_phase = phase;
    s_bp_base_ticks = subsystem5_ticks_now();
    s_bp_clear_count = 0U;
    s_bp_line_at = 0U;
    subsystem5_abort_pending_echo();
    tm_printf((UB*)"[S5] bypass: %s\n", s_bp_name[phase]);

    switch (phase)
    {
    case BP_TURN_OUT:                    /* toward the passing side */
    case BP_ALIGN:
        subsystem5_servo_set(90U);
        subsystem5_drive(t, -t);
        break;
    case BP_TURN_BACK:                   /* toward the obstacle / line side */
    case BP_TURN_IN:
        subsystem5_servo_set(90U);
        subsystem5_drive(-t, t);
        break;
    case BP_PASS:                        /* look at the obstacle, 60 deg off */
        subsystem5_servo_set((subsystem5_bp_dir() > 0) ? 150U : 30U);
        s_poll.bp_seen = 0U;
        s_bp_passed_at = 0U;
        subsystem5_drive(v, v);
        break;
    default:                             /* OUT, SEEK: look ahead */
        subsystem5_servo_set(90U);
        subsystem5_drive(v, v);
        break;
    }
}

static void subsystem5_bypass_fail(void)
{
    b2_stop();
    subsystem5_abort_pending_echo();
    subsystem5_fault(subsystem5_FAULT_BYPASS);
    tm_printf((UB*)"[S5] bypass failed during \"%s\", stopped\n",
              s_bp_name[s_poll.bp_phase]);
}

/* How far sideways the car must move: the passing-side edge's offset from
 * the car's centre line, plus half the car and a margin. */
static uint32_t subsystem5_out_distance_mm(void)
{
    const subsystem5_profile_t *p = &s_status.profile;
    float edge_mm;
    float need_mm;
    float a_deg;
    float sin_a;

    if ((p->right_edge_mm != 0U) && (p->left_edge_mm != 0U))
    {
        edge_mm = (subsystem5_bp_dir() > 0) ?
            ((float)p->right_edge_mm * subsystem5_cosd(
                (float)p->right_edge_deg + (float)subsystem5_BEAM_EDGE_DEG)) :
            -((float)p->left_edge_mm * subsystem5_cosd(
                (float)p->left_edge_deg - (float)subsystem5_BEAM_EDGE_DEG));
    }
    else
    {
        edge_mm = (float)p->approximate_width_mm * 0.5f;
    }
    if (edge_mm < 0.0f) { edge_mm = 0.0f; }
    need_mm = edge_mm + (float)subsystem5_CAR_HALF_WIDTH_MM +
              (float)subsystem5_BYPASS_MARGIN_MM;

    /* Angle actually turned is only roughly known: estimate from the count,
       never below 25 degrees (a long but safe OUT leg). */
    a_deg = ((float)s_bp_turn_ticks * 10.0f) /
            (float)subsystem5_TICKS_PER_DEG_X10;
    if (a_deg < 25.0f) { a_deg = 25.0f; }
    if (a_deg > 90.0f) { a_deg = 90.0f; }
    sin_a = subsystem5_cosd(90.0f - a_deg);
    need_mm /= sin_a;
    if (need_mm < 120.0f) { need_mm = 120.0f; }
    if (need_mm > (float)subsystem5_BYPASS_OUT_MAX_MM)
    {
        need_mm = (float)subsystem5_BYPASS_OUT_MAX_MM;
    }
    return (uint32_t)need_mm;
}

static void subsystem5_bypass_begin(void)
{
    s_poll.bp_left = (s_status.profile.candidate_side == subsystem5_SIDE_LEFT) ?
                     1U : 0U;
    s_bp_turn_ticks = 0U;
    s_bp_clear_at = 0U;
    s_bp_out_mm = 0U;
    s_bp_started_us = subsystem5_now_us();
    s_status.state = subsystem5_STATE_BYPASS;
    s_status.stop_required = 0U;  /* This motion is authorized: VC must not stop it. */
    tm_printf((UB*)"[S5] bypassing on the %s\n", (subsystem5_bp_dir() > 0) ? "right" : "left");
    subsystem5_bp_enter(BP_TURN_OUT);
}

static void subsystem5_bypass_step(uint32_t now_us)
{
    const uint32_t done = subsystem5_ticks_now() - s_bp_base_ticks;
    subsystem5_sample_t r;
    bool have = false;
    b2_status_t st;

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

    if ((s_poll.range_active == 0U) &&
        ((int32_t)(now_us - s_servo_ready_us) >= 0))
    {
        (void)subsystem5_ping_begin(now_us);
    }
    r.angle_deg = s_servo_angle;
    if (subsystem5_ping_result(now_us, &r) && (r.valid != 0U))
    {
        have = true;
        if (s_servo_angle == 90U)
        {
            s_status.front_valid = (r.distance_mm != subsystem5_FAR_MM) ? 1U : 0U;
            s_status.front_mm = r.distance_mm;
        }
    }

    /* Driving straight while looking ahead: never into something. */
    if (have && (s_servo_angle == 90U) && (r.distance_mm <= subsystem5_URGENT_MM) &&
        ((s_poll.bp_phase == BP_OUT) || (s_poll.bp_phase == BP_SEEK)))
    {
        subsystem5_bypass_fail();
        return;
    }

    switch (s_poll.bp_phase)
    {
    case BP_TURN_OUT:
    {
        const uint32_t clear_mm = (uint32_t)s_status.profile.closest_mm +
                                  subsystem5_BYPASS_CLEAR_EXTRA_MM;
        if (have)
        {
            s_bp_clear_count = (r.distance_mm > clear_mm) ?
                (uint8_t)(s_bp_clear_count + 1U) : 0U;
        }
        if ((s_bp_clear_at == 0U) && (s_bp_clear_count >= 2U) &&
            (done >= subsystem5_deg_to_ticks(subsystem5_BYPASS_MIN_TURN_DEG)))
        {
            s_bp_clear_at = done;
        }
        if ((s_bp_clear_at != 0U) &&
            (done >= (s_bp_clear_at +
                      subsystem5_deg_to_ticks(subsystem5_BYPASS_EXTRA_TURN_DEG))))
        {
            s_bp_turn_ticks = done;
            s_bp_out_mm = subsystem5_out_distance_mm();
            tm_printf((UB*)"[S5] turned %u ticks (~%u deg), out leg %u mm\n",
                      (unsigned)done,
                      (unsigned)((done * 10U) / subsystem5_TICKS_PER_DEG_X10),
                      (unsigned)s_bp_out_mm);
            subsystem5_bp_enter(BP_OUT);
        }
        else if (done > subsystem5_deg_to_ticks(subsystem5_BYPASS_MAX_TURN_DEG))
        {
            subsystem5_bypass_fail();    /* never saw past it */
        }
        break;
    }
    case BP_OUT:
        if (done >= subsystem5_mm_to_ticks(s_bp_out_mm))
        {
            subsystem5_bp_enter(BP_TURN_BACK);
        }
        break;
    case BP_TURN_BACK:
        if (done >= s_bp_turn_ticks)
        {
            subsystem5_bp_enter(BP_PASS);
        }
        break;
    case BP_PASS:
        if (have)
        {
            if (r.distance_mm <= subsystem5_OBSTACLE_ZONE_MM)
            {
                s_poll.bp_seen = 1U;
                s_bp_clear_count = 0U;
            }
            else if (s_poll.bp_seen != 0U)
            {
                s_bp_clear_count++;
            }
        }
        if ((s_bp_passed_at == 0U) &&
            (((s_poll.bp_seen != 0U) && (s_bp_clear_count >= 2U)) ||
             ((s_poll.bp_seen == 0U) &&
              (done >= subsystem5_mm_to_ticks(subsystem5_BYPASS_PASS_BLIND_MM)))))
        {
            s_bp_passed_at = (done > 0U) ? done : 1U;
        }
        if ((s_bp_passed_at != 0U) &&
            (done >= (s_bp_passed_at +
                      subsystem5_mm_to_ticks(subsystem5_CAR_LENGTH_MM))))
        {
            subsystem5_bp_enter(BP_TURN_IN);
        }
        else if (done > subsystem5_mm_to_ticks(subsystem5_BYPASS_PASS_MAX_MM))
        {
            subsystem5_bypass_fail();    /* still beside it: too long */
        }
        break;
    case BP_TURN_IN:
        if (done >= s_bp_turn_ticks)
        {
            subsystem5_bp_enter(BP_SEEK);
        }
        break;
    case BP_SEEK:
        /* The line sensors are ahead of the wheels: when they find the
           line, carry on until the wheels are on it, so the pivot leaves
           the sensors on the line. */
        if ((s_bp_line_at == 0U) &&
            (get_line_barcode_data_safe().line_detected != 0U))
        {
            s_bp_line_at = (done > 0U) ? done : 1U;
        }
        if ((s_bp_line_at != 0U) &&
            (done >= (s_bp_line_at +
                      subsystem5_mm_to_ticks(subsystem5_SENSOR_TO_AXLE_MM))))
        {
            subsystem5_bp_enter(BP_ALIGN);
        }
        else if ((s_bp_line_at == 0U) &&
                 (done > subsystem5_mm_to_ticks(s_bp_out_mm +
                                                subsystem5_BYPASS_SEEK_EXTRA_MM)))
        {
            subsystem5_bypass_fail();    /* drove past where the line was */
        }
        break;
    default:                              /* BP_ALIGN */
        if (done >= s_bp_turn_ticks)
        {
            b2_stop();
            tm_printf((UB*)"[S5] bypass done in %u ms, back on the line\n",
                      (unsigned)(subsystem5_elapsed(s_bp_started_us, now_us) /
                                 1000U));
            subsystem5_reset_for_monitor();
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
    cmd.goal_speed_mm_s = subsystem5_REVERSE_SPEED_MM_S;
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