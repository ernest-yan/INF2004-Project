/*
 * Subsystem 5: HC-SR04 + SG90 servo, RP2040/Pico SDK.
 * All motion commands are advisory events for VehicleController.
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

#define subsystem5_ECHO_WAIT_RISE       (1U)
#define subsystem5_ECHO_WAIT_FALL       (2U)
#define subsystem5_ECHO_COMPLETE        (3U)

static const uint8_t s_coarse_angles[subsystem5_COARSE_COUNT] =
{
    30U, 60U, 90U, 120U, 150U
};

/* PORT: echo variables are now touched only by the poll task (no interrupt). */
static volatile uint8_t s_echo_phase;
static volatile uint32_t s_echo_rise_us;
static volatile uint32_t s_echo_pulse_us;

/* Poll-task-owned state; other tasks use command flags and snapshot API. */
static subsystem5_status_t s_status;
static subsystem5_status_t s_snapshot;
static subsystem5_scan_samples_t s_samples;
static subsystem5_event_publish_fn s_publish;
static void *s_publish_context;
static uint8_t s_range_active;
static uint8_t s_scan_index;
static uint8_t s_scan_retries;
static uint8_t s_monitor_invalid;
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
    event.type = type;
    event.fault = fault;
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
        s_status.fault = subsystem5_FAULT_EVENT_DELIVERY;
        s_status.stop_required = 1U;
        s_status.state = subsystem5_STATE_FAULT;
    }
}

static void subsystem5_fault(subsystem5_fault_t fault)
{
    s_status.stop_required = 1U;
    s_status.fault = fault;
    s_status.state = subsystem5_STATE_FAULT;
    subsystem5_send_event(subsystem5_EVENT_FAULT, fault);
}

/* PORT: replaces the GPIO edge interrupt.  Polls the configured ECHO GPIO with 1 us time stamps
 * until the echo pulse has risen and fallen, or the timeout passes.  Same
 * state machine as the old subsystem5_echo_gpio_irq(), driven by the pin
 * level instead of edge events. */
static void subsystem5_echo_measure(void)
{
    for (;;)
    {
        const uint32_t t = subsystem5_now_us();
        const UINT level = gpio_get_val(subsystem5_ECHO_GPIO);

        if (subsystem5_elapsed(s_range_started_us, t) >= subsystem5_ECHO_TIMEOUT_US)
        {
            return; /* No complete echo: subsystem5_ping_result() reports invalid. */
        }

        if ((s_echo_phase == subsystem5_ECHO_WAIT_RISE) && (level != 0U))
        {
            s_echo_rise_us = t;
            s_echo_phase = subsystem5_ECHO_WAIT_FALL;
        }
        else if ((s_echo_phase == subsystem5_ECHO_WAIT_FALL) && (level == 0U))
        {
            s_echo_pulse_us = subsystem5_elapsed(s_echo_rise_us, t);
            s_echo_phase = subsystem5_ECHO_COMPLETE;
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
    if ((s_range_active != 0U) ||
        (subsystem5_elapsed(s_last_ping_us, now_us) < subsystem5_PING_GAP_US))
    {
        return false;
    }

    s_echo_phase = subsystem5_ECHO_WAIT_RISE;
    s_echo_pulse_us = 0U;
    s_echo_rise_us = 0U;

    s_range_active = 1U;
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

    if ((s_range_active == 0U) || (result == NULL))
    {
        return false;
    }

    /* PORT: no interrupt-masked copy needed; only this task touches these. */
    phase = s_echo_phase;
    if (phase == subsystem5_ECHO_COMPLETE)
    {
        pulse_us = s_echo_pulse_us;
        s_echo_phase = 0U;
    }
    else if (subsystem5_elapsed(s_range_started_us, now_us) >= subsystem5_ECHO_TIMEOUT_US)
    {
        s_echo_phase = 0U;
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

    result->valid = 0U;
    result->distance_mm = subsystem5_UNKNOWN_MM;
    if ((phase == subsystem5_ECHO_COMPLETE) &&
        (pulse_us >= 116U) && (pulse_us <= 23200U))
    {
        /* Datasheet: distance_cm = echo_us / 58; integer mm. */
        result->distance_mm = (uint16_t)((pulse_us * 10U) / 58U);
        result->valid = 1U;
    }
    s_range_active = 0U;
    return true;
}

static void subsystem5_abort_pending_echo(void)
{
    s_echo_phase = 0U;      /* PORT: no interrupt masking needed */
    s_range_active = 0U;
}

static void subsystem5_reset_for_monitor(void)
{
    s_status.fault = subsystem5_FAULT_NONE;
    s_status.stop_required = 0U;
    s_status.scan_complete = 0U;
    s_status.state = subsystem5_STATE_MONITOR;
    s_status.front_valid = 0U;
    s_monitor_invalid = 0U;
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

    for (index = 0U; index < subsystem5_COARSE_COUNT; index++)
    {
        const subsystem5_sample_t *point = &s_samples.coarse[index];
        if ((point->valid != 0U) &&
            (point->distance_mm <= subsystem5_OBSTACLE_ZONE_MM) &&
            (point->distance_mm < closest_mm))
        {
            closest_mm = point->distance_mm;
            focus = (uint8_t)point->angle_deg;
        }
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

    /* Shift the window at scan-arc limits to avoid duplicate angles. */
    {
        const uint8_t first = subsystem5_clamp_angle(
            (int16_t)((focus > 20U) ? (focus - 20U) : 0U));
        const uint8_t start = (first > 110U) ? 110U : first;
        for (index = 0U; index < subsystem5_FINE_COUNT; index++)
        {
            s_fine_angles[index] = (uint8_t)(start + (index * 10U));
        }
    }

    s_scan_index = 0U;
    s_status.state = subsystem5_STATE_FINE;
    subsystem5_servo_set(s_fine_angles[0]);
}

static void subsystem5_scan_finish(void)
{
    s_status.profile = subsystem5_profile_analyze(&s_samples,
                                          subsystem5_OBSTACLE_ZONE_MM,
                                          subsystem5_SIDE_CLEAR_MM);
    if ((s_status.profile.profile_valid == 0U) ||
        (s_status.profile.obstacle_found == 0U))
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

    if (s_status.profile.candidate_side == subsystem5_SIDE_NONE)
    {
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

    if ((s_range_active == 0U) &&
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
            s_status.state = subsystem5_STATE_WAIT_STOP;
            subsystem5_send_event(subsystem5_EVENT_STOP_REQUIRED, subsystem5_FAULT_NONE);
            subsystem5_snapshot_update();
            return;
        }

        if ((s_range_active == 0U) &&
            ((int32_t)(now_us - s_servo_ready_us) >= 0))
        {
            (void)subsystem5_ping_begin(now_us);
        }
        front.angle_deg = 90U;
        if (subsystem5_ping_result(now_us, &front))
        {
            s_status.front_valid = front.valid;
            s_status.front_mm = front.distance_mm;
            if (front.valid == 0U)
            {
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
                if (front.distance_mm <= subsystem5_DETECT_MM)
                {
                    /* Request stopping on the FIRST close valid reading;
                     * do not wait for the slow servo scan while moving. */
                    s_status.stop_required = 1U;
                    s_stop_request_us = now_us;
                    s_scan_retries = 0U;
                    s_status.state = subsystem5_STATE_WAIT_STOP;
                    subsystem5_send_event(subsystem5_EVENT_STOP_REQUIRED, subsystem5_FAULT_NONE);
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
    s_range_active = 0U;
    s_monitor_invalid = 0U;
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

    s_echo_phase = 0U;
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