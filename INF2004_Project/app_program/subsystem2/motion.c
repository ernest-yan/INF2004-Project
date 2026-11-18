/* IS33 Buddy 2. One motor-output owner, shared encoder IRQ dispatcher. */
#include "motion.h"
#include "math.h"
#include <limits.h>
/* PORT: no Pico SDK in this firmware (the kernel does its own start-up and the
 * SDK runtime is not linked).  Everything SDK-specific now goes through
 * libbsp, the kernel interrupt API and ../app_hw.h.  <string.h> is not used:
 * the kernel's tk/syslib.h has its own size_t.  Every change is marked PORT. */
#include "../app_hw.h"
#include "tkernel_adapter.h"

/* PORT: RP2040 IO_BANK0 interrupt registers (core 0 = PROC0). */
#define B2_IO_IRQ_BANK0       (13U)
#define B2_IO_INTR0           (IO_BANK0_BASE + 0x0F0U)
#define B2_IO_PROC0_INTE0     (IO_BANK0_BASE + 0x100U)
#define B2_IO_PROC0_INTS0     (IO_BANK0_BASE + 0x120U)
#define B2_GPIO_EDGE_HIGH     (0x8U)  /* per-pin nibble: bit3 = rising edge */
#define B2_IRQ_BIT(gpio)      (B2_GPIO_EDGE_HIGH << (4U * (gpio)))
#define GPIO_IRQ_EDGE_RISE    (0x8U)  /* same encoding as the Pico SDK */
/* PORT: PWM DIV register of a slice; 0x10 = divide by 1.0 */
#define B2_PWM_DIV(slice)     (PWM_BASE + 0x04U + ((slice) * 0x14U))
#define B2_PWM_SLICE(gpio)    (((gpio) >> 1U) & 7U)

/* PORT: one lock for all shared motion state: masks the encoder IRQ on this
 * core AND excludes callers on the other core (S4 urgent brake, controller). */
static app_lock_t s_lock;

#define B2_LOOP_MS            (10U)
#define B2_MIN_TICKS          (1U)
#define B2_STOPPED_STABLE_MS  (200U)
#define B2_MAX_DUTY          (1000U)
#define B2_MIN_OUTPUT        (0U)
#define B2_INTEGRAL_LIMIT     (8000)   /* PI integral clamp (anti-windup) */
#define B2_INTEGRAL_RESET_MM_S (20)    /* request change that resets it */
#define B2_GOAL_START_US     (100000U)

/* CR1: a validated command packed into two words (8 bytes, was 16).
 * Speeds offset-binary (Barr C: no signed bit-fields). */
typedef struct
{
    uint32_t type : 3;              /* b2_cmd_type_t */
    uint32_t left_ob : 10;          /* left_mm_s + B2_SPEED_OFFSET */
    uint32_t right_ob : 10;
    uint32_t goal_speed : 9;        /* 0..511 mm/s */
    uint32_t distance_mm : 13;      /* 0..8191 */
    uint32_t turn_deg : 9;          /* 0..360 */
    uint32_t timeout_10ms : 10;     /* 0..1023 x 10 ms */
} b2_cmd_packed_t;

/* CR1: flags grouped by WHO WRITES them. A bit-field write is a
 * read-modify-write of the whole byte, so two writers on different tasks
 * or cores could undo each other's change. s_shared is written by other
 * tasks too: every write holds s_lock. s_own is written by the motion
 * task only. Reads of a single bit are safe without the lock. */
static struct
{
    uint8_t ready : 1;
    uint8_t pending : 1;            /* s_mailbox holds a new command */
    uint8_t stop_event_pending : 1;
    uint8_t brake_event_pending : 1;
    uint8_t : 4;
} s_shared;
static struct
{
    uint8_t poll_started : 1;       /* replaces s_last_poll_us == 0 */
    uint8_t was_moving : 1;
    uint8_t event_pending : 1;
    uint8_t : 5;
} s_own;

static const b2_config_t *s_cfg;    /* CR1: points at the caller's const config */
static b2_status_t s_status;
static b2_event_publish_fn s_publish;
static void *s_publish_ctx;
/* Written by the encoder ISR: full aligned words (single-instruction store
 * on Cortex-M0+), never packed. Read by the task only inside s_lock. */
static volatile uint32_t s_irq_count[2];
static volatile uint32_t s_last_edge_us[2]; /* CR1: was uint64_t, which the
                                               ISR stored in two halves */
static b2_cmd_packed_t s_mailbox;
static b2_cmd_packed_t s_active;
static uint32_t s_goal_ticks;
static uint32_t s_start_ticks[2];
static uint32_t s_prev_ticks[2];
static int16_t s_requested[2];      /* -511..511 mm/s */
static int16_t s_integral[2];       /* clamped to +-B2_INTEGRAL_LIMIT */
static uint32_t s_goal_started_us;
static uint32_t s_wheel_started_us[2];  /* last time this wheel went 0 -> moving */
static uint32_t s_last_poll_us;
static uint32_t s_last_stopped_check_us;
static uint16_t s_pwm_wrap;
static volatile uint8_t s_command_generation; /* equality only; under s_lock */
static int32_t s_signed_encoder_ticks[2];
static b2_event_t s_pending_event;

/* PORT: kernel interrupt handler for IO_IRQ_BANK0 (replaces the SDK's GPIO
 * callback).  It is the team's SINGLE GPIO-bank handler: if another subsystem
 * ever needs a GPIO interrupt it must be routed from here.  Only GP6/GP7 are
 * enabled, so only their rising-edge bits are checked and acknowledged. */
static void b2_io_bank0_isr(UINT intno)
{
    const uint32_t mask = B2_IRQ_BIT(B2_ENCODER_LEFT_GP) |
                          B2_IRQ_BIT(B2_ENCODER_RIGHT_GP);
    const uint32_t pending = in_w(B2_IO_PROC0_INTS0) & mask;
    (void)intno;
    out_w(B2_IO_INTR0, pending);      /* edge bits are write-1-to-clear */
    if ((pending & B2_IRQ_BIT(B2_ENCODER_LEFT_GP)) != 0U)
    {
        b2_encoder_gpio_irq(B2_ENCODER_LEFT_GP, GPIO_IRQ_EDGE_RISE);
    }
    if ((pending & B2_IRQ_BIT(B2_ENCODER_RIGHT_GP)) != 0U)
    {
        b2_encoder_gpio_irq(B2_ENCODER_RIGHT_GP, GPIO_IRQ_EDGE_RISE);
    }
}

static int32_t b2_abs32(int32_t a)
{
    return (a < 0) ? -a : a;
}
static int32_t b2_clamp(int32_t n, int32_t minimum, int32_t maximum)
{
    if (n < minimum) { return minimum; }
    if (n > maximum) { return maximum; }
    return n;
}
/* Signed speed -> 10-bit offset-binary field, saturating. */
static uint32_t b2_speed_field(int32_t mm_s)
{
    return (uint32_t)b2_clamp(mm_s + B2_SPEED_OFFSET, 0, B2_SPEED_FIELD_MAX);
}
/* Wrap-safe "a is later than b" for 32-bit microsecond times. */
static bool b2_time_after(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) > 0;
}
/* Call with s_lock held. */
static int32_t b2_wheel_speed_locked(uint32_t i)
{
    return b2_speed_value((i == 0U) ? s_status.left_speed_ob :
                                      s_status.right_speed_ob);
}
static void b2_set_wheel_speed_locked(uint32_t i, int32_t mm_s)
{
    if (i == 0U) { s_status.left_speed_ob = b2_speed_field(mm_s); }
    else         { s_status.right_speed_ob = b2_speed_field(mm_s); }
}
/* The caller has already range-checked cmd (b2_submit_command). */
static b2_cmd_packed_t b2_pack_command(const b2_command_t *cmd)
{
    b2_cmd_packed_t p;
    p.type = (uint32_t)cmd->type;
    p.left_ob = b2_speed_field(cmd->left_mm_s);
    p.right_ob = b2_speed_field(cmd->right_mm_s);
    p.goal_speed = cmd->goal_speed_mm_s;
    p.distance_mm = cmd->distance_mm;
    p.turn_deg = cmd->turn_degrees;
    /* Round up, so a goal never times out earlier than asked. */
    p.timeout_10ms = ((uint32_t)cmd->timeout_ms + (B2_TIMEOUT_UNIT_MS - 1U)) /
                     B2_TIMEOUT_UNIT_MS;
    return p;
}
/* Always call with interrupts disabled; both channels LOW means BRAKE. */
static void b2_write_wheels_locked(int32_t a, int32_t b)
{
    const uint32_t pins[2][2] = {
        {B2_MOTOR_LEFT_A_GP, B2_MOTOR_LEFT_B_GP},
        {B2_MOTOR_RIGHT_A_GP, B2_MOTOR_RIGHT_B_GP}
    };
    const int32_t outputs[2] = {a, b};
    for (uint32_t i = 0U; i < 2U; ++i)
    {
        const uint32_t duty = (uint32_t)b2_abs32(outputs[i]);
        const uint32_t safe = (duty > B2_MAX_DUTY) ? B2_MAX_DUTY : duty;
        const uint16_t level = (uint16_t)(((uint64_t)s_pwm_wrap * safe) /
                                          B2_MAX_DUTY);
        (void)pwm_set_cc(pins[i][0], (outputs[i] > 0) ? level : 0U);
        (void)pwm_set_cc(pins[i][1], (outputs[i] < 0) ? level : 0U);
    }
    s_status.left_pwm_permil = (uint32_t)b2_clamp(b2_abs32(a), 0,
                                                  (int32_t)B2_MAX_DUTY);
    s_status.right_pwm_permil = (uint16_t)b2_clamp(b2_abs32(b), 0,
                                                   (int32_t)B2_MAX_DUTY);
}
static void b2_zero_outputs_locked(void)
{
    b2_write_wheels_locked(0, 0);
    s_requested[0] = 0;
    s_requested[1] = 0;
    s_integral[0] = 0;
    s_integral[1] = 0;
}
/* Motion task only. */
static void b2_queue_event(b2_event_type_t type, b2_fault_t fault)
{
    s_pending_event.type = (uint8_t)type;
    s_pending_event.fault = (uint8_t)fault;
    s_own.event_pending = 1U;
}
static void b2_fault_now(b2_fault_t fault)
{
    const UINT irq = app_lock(&s_lock);
    b2_zero_outputs_locked();
    s_status.state = B2_FAULT;
    s_status.fault_mask |= B2_FAULT_BIT(fault);
    s_status.brake_latched = 1U;
    s_shared.pending = 0U;
    ++s_command_generation;
    app_unlock(&s_lock, irq);
    b2_queue_event(B2_EVENT_FAULT, fault);
}
static void b2_deliver_event(void)
{
    if (s_own.event_pending != 0U)
    {
        const b2_event_t event = s_pending_event;
        s_own.event_pending = 0U;
        if ((s_publish != NULL) && !s_publish(&event, s_publish_ctx))
        {
            /* An event queue must never silently lose critical status. */
            b2_fault_now(B2_FAULT_EVENT_LOSS);
        }
    }
}
static void b2_configure_pin(unsigned int gpio)
{
    (void)pwm_set_cc(gpio, 0U);       /* PORT: was pwm_set_gpio_level() */
    (void)pwm_set_pin(gpio);          /* PORT: was gpio_set_function(PWM) */
}
bool b2_init(const b2_config_t *config, b2_event_publish_fn publish, void *ctx)
{
    if ((config == NULL) || (config->ticks_per_wheel_rev == 0U) ||
        (config->wheel_circumference_mm == 0U) ||
        (config->wheel_track_mm == 0U) ||
        (config->max_speed_mm_s == 0U) ||
        (config->max_speed_mm_s > B2_SPEED_LIMIT_MM_S) ||   /* CR1 field */
        (config->encoder_stall_ms > 60000U) ||  /* x1000 must fit 32 bits */
        (config->max_pwm_permil == 0U) ||
        (config->max_pwm_permil > B2_MAX_DUTY) ||
        (config->encoder_stall_ms == 0U) ||
        (config->pwm_frequency_hz < 1000U))
    {
        return false;
    }
    {   /* PORT: memset() replaced (size_t clash with the kernel headers) */
        static const b2_status_t zero_status;
        s_status = zero_status;
    }
    APP_LOCK_INIT(s_lock);
    s_cfg = config;
    s_publish = publish;
    s_publish_ctx = ctx;
    s_status.state = B2_READY;
    s_status.left_speed_ob = b2_speed_field(0);
    s_status.right_speed_ob = b2_speed_field(0);
    s_command_generation = 0U;
    s_signed_encoder_ticks[0] = 0;
    s_signed_encoder_ticks[1] = 0;
    s_irq_count[0] = 0U;
    s_irq_count[1] = 0U;
    s_last_edge_us[0] = 0U;
    s_last_edge_us[1] = 0U;
    {   /* the motion task has not started: no other writer yet */
        const UINT irq = app_lock(&s_lock);
        s_shared.ready = 0U;
        s_shared.pending = 0U;
        s_shared.stop_event_pending = 0U;
        s_shared.brake_event_pending = 0U;
        app_unlock(&s_lock, irq);
    }
    s_own.poll_started = 0U;
    s_own.was_moving = 0U;
    s_own.event_pending = 0U;
    s_last_poll_us = 0U;
    s_last_stopped_check_us = 0U;
    {
        const uint32_t wrap = (APP_CLK_SYS_HZ / config->pwm_frequency_hz) - 1U;
        if ((wrap < 100U) || (wrap > 65535U))   /* PORT */
        {
            return false;
        }
        s_pwm_wrap = (uint16_t)wrap;
    }
    /* PORT: PWM block is shared with S5's servo: release reset, never reset. */
    app_release_reset(RESETS_RESET_PWM);
    const unsigned int pins[4] = {B2_MOTOR_LEFT_A_GP, B2_MOTOR_LEFT_B_GP,
                                  B2_MOTOR_RIGHT_A_GP, B2_MOTOR_RIGHT_B_GP};
    const unsigned int slice_l = B2_PWM_SLICE(B2_MOTOR_LEFT_A_GP);
    const unsigned int slice_r = B2_PWM_SLICE(B2_MOTOR_RIGHT_A_GP);
    out_w(B2_PWM_DIV(slice_l), 0x10U);                 /* clkdiv 1.0 */
    out_w(B2_PWM_DIV(slice_r), 0x10U);
    (void)pwm_set_wrap(B2_MOTOR_LEFT_A_GP, s_pwm_wrap);
    (void)pwm_set_wrap(B2_MOTOR_RIGHT_A_GP, s_pwm_wrap);
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        b2_configure_pin(pins[i]);
    }
    (void)pwm_set_enabled(B2_MOTOR_LEFT_A_GP, TRUE);
    (void)pwm_set_enabled(B2_MOTOR_RIGHT_A_GP, TRUE);

    /* PORT: encoder inputs + rising-edge interrupt through the kernel.
     * Encoder pull-ups/polarity depend on verified harness: pads left as is
     * (RP2040 reset default = pull-down). Must run on the core that should
     * take the IRQ (core 0): b2_init() is called from the core-0 motion task. */
    (void)gpio_set_pin(B2_ENCODER_LEFT_GP, GPIO_MODE_IN);
    (void)gpio_set_pin(B2_ENCODER_RIGHT_GP, GPIO_MODE_IN);
    {
        T_DINT dint;
        const uint32_t mask = B2_IRQ_BIT(B2_ENCODER_LEFT_GP) |
                              B2_IRQ_BIT(B2_ENCODER_RIGHT_GP);
        dint.intatr = TA_HLNG;
        dint.inthdr = (FP)b2_io_bank0_isr;
        if (tk_def_int(B2_IO_IRQ_BANK0, &dint) != E_OK)
        {
            return false;
        }
        out_w(B2_IO_INTR0, mask);               /* drop stale edges */
        set_w(B2_IO_PROC0_INTE0, mask);         /* enable GP6/GP7 rising */
        ClearInt(B2_IO_IRQ_BANK0);
        EnableInt(B2_IO_IRQ_BANK0, S2_ENC_INTPRI);
    }
    {
        const UINT irq = app_lock(&s_lock);
        s_shared.ready = 1U;
        app_unlock(&s_lock, irq);
    }
    return true;
}
void b2_encoder_gpio_irq(unsigned int gpio, uint32_t events)
{
    if ((events & GPIO_IRQ_EDGE_RISE) != 0U)
    {
        const uint32_t i = (gpio == B2_ENCODER_LEFT_GP) ? 0U : 1U;
        if ((gpio == B2_ENCODER_LEFT_GP) || (gpio == B2_ENCODER_RIGHT_GP))
        {
            ++s_irq_count[i];
            s_last_edge_us[i] = app_time_us_32();
        }
    }
}
bool b2_submit_command(const b2_command_t *cmd)
{
    if ((s_shared.ready == 0U) || (cmd == NULL)) { return false; }
    if (cmd->type > B2_CMD_TURN_RIGHT) { return false; }
    /* CR1: every field is range-checked here, before packing, so a request
     * that does not fit its bit-field is refused, never truncated. */
    if (cmd->type == B2_CMD_SPEED)
    {
        if ((b2_abs32(cmd->left_mm_s) > s_cfg->max_speed_mm_s) ||
            (b2_abs32(cmd->right_mm_s) > s_cfg->max_speed_mm_s))
        { return false; }
    }
    else if ((cmd->goal_speed_mm_s == 0U) ||
             (cmd->goal_speed_mm_s > s_cfg->max_speed_mm_s) ||
             (cmd->timeout_ms == 0U) ||
             (cmd->timeout_ms > B2_TIMEOUT_MAX_MS) ||
             (((cmd->type == B2_CMD_MOVE_FORWARD) ||
               (cmd->type == B2_CMD_MOVE_BACKWARD)) &&
              ((cmd->distance_mm == 0U) ||
               (cmd->distance_mm > B2_DISTANCE_MAX_MM))) ||
             (((cmd->type == B2_CMD_TURN_LEFT) ||
               (cmd->type == B2_CMD_TURN_RIGHT)) &&
              ((cmd->turn_degrees == 0U) ||
               (cmd->turn_degrees > B2_TURN_MAX_DEG))))
    { return false; }
    const b2_cmd_packed_t packed = b2_pack_command(cmd);
    const UINT irq = app_lock(&s_lock);
    const bool allowed = (s_status.brake_latched == 0U) &&
        (s_status.state != B2_FAULT) &&
        ((s_status.state == B2_READY) ||
         ((s_status.state == B2_TRACK_SPEED) && (cmd->type == B2_CMD_SPEED))) &&
        ((s_shared.pending == 0U) || ((cmd->type == B2_CMD_SPEED) &&
                                      (s_mailbox.type == B2_CMD_SPEED)));
    if (allowed)
    {
        s_mailbox = packed;
        s_shared.pending = 1U;
    }
    app_unlock(&s_lock, irq);
    return allowed;
}
void b2_stop(void)
{
    const UINT irq = app_lock(&s_lock);
    if ((s_shared.ready != 0U) && (s_status.brake_latched == 0U))
    {
        b2_zero_outputs_locked();
        s_status.state = B2_READY;
        s_shared.pending = 0U;
        ++s_command_generation;
        s_shared.stop_event_pending = 1U;
        s_last_stopped_check_us = app_time_us_32();
    }
    app_unlock(&s_lock, irq);
}
void b2_emergency_brake(void)
{
    if (s_shared.ready == 0U) { return; }
    const UINT irq = app_lock(&s_lock);
    b2_zero_outputs_locked();
    s_status.brake_latched = 1U;
    s_status.state = B2_BRAKED;
    s_shared.pending = 0U;
    ++s_command_generation;
    s_shared.brake_event_pending = 1U;
    app_unlock(&s_lock, irq);
}
bool b2_authorized_reset_while_stopped(void)
{
    if (s_shared.ready == 0U) { return false; }
    const UINT irq = app_lock(&s_lock);
    /* Encoder silence supports but cannot prove physical standstill. */
    const uint32_t now = app_time_us_32();
    if (((s_last_edge_us[0] != 0U) &&
         ((now - s_last_edge_us[0]) < 200000U)) ||
        ((s_last_edge_us[1] != 0U) &&
         ((now - s_last_edge_us[1]) < 200000U)))
    {
        app_unlock(&s_lock, irq);
        return false;
    }
    /* Must not release the latch merely due to a new START command. */
    b2_zero_outputs_locked();
    s_status.brake_latched = 0U;
    s_status.fault_mask = 0U;
    s_status.state = B2_READY;
    s_shared.pending = 0U;
    ++s_command_generation;
    s_shared.stop_event_pending = 0U;
    s_shared.brake_event_pending = 0U;
    app_unlock(&s_lock, irq);
    return true;
}
void b2_get_status(b2_status_t *out)
{
    if (out != NULL)
    {
        const UINT irq = app_lock(&s_lock);
        *out = s_status;
        app_unlock(&s_lock, irq);
    }
}
bool b2_goal_start(b2_goal_t *goal, const b2_command_t *cmd)
{
    b2_status_t st;
    b2_get_status(&st);
    goal->goals_done_at_start = st.goals_done;
    goal->started_us = app_time_us_32();
    goal->seen_running = 0U;
    return b2_submit_command(cmd);
}
b2_goal_result_t b2_goal_check(b2_goal_t *goal)
{
    b2_status_t st;
    b2_get_status(&st);
    if (st.goals_done != goal->goals_done_at_start)
    {
        return B2_GOAL_DONE;
    }
    if (st.state == B2_EXECUTE_GOAL)
    {
        goal->seen_running = 1U;
        return B2_GOAL_RUNNING;
    }
    /* Still READY shortly after the start: waiting for the next b2_poll(). */
    if ((st.state == B2_READY) && (goal->seen_running == 0U) &&
        ((app_time_us_32() - goal->started_us) < B2_GOAL_START_US))
    {
        return B2_GOAL_RUNNING;
    }
    return B2_GOAL_FAILED;
}
/* Call with s_lock held. cmd has been range-checked and packed. */
static void b2_start_command(const b2_cmd_packed_t *cmd, uint32_t tick_l,
                             uint32_t tick_r, uint32_t now)
{
    const bool continuous_speed = (cmd->type == B2_CMD_SPEED) &&
                                  (s_status.state == B2_TRACK_SPEED);
    const int32_t previous[2] = {s_requested[0], s_requested[1]};
    const int32_t goal_speed = (int32_t)cmd->goal_speed;
    int32_t request[2];
    s_active = *cmd;
    if (!continuous_speed)
    {
        s_integral[0] = 0;
        s_integral[1] = 0;
        s_goal_started_us = now;
    }
    s_start_ticks[0] = tick_l;
    s_start_ticks[1] = tick_r;
    s_goal_ticks = 0U;
    if (cmd->type == B2_CMD_SPEED)
    {
        request[0] = b2_speed_value(cmd->left_ob);
        request[1] = b2_speed_value(cmd->right_ob);
        s_status.state = B2_TRACK_SPEED;
    }
    else
    {
        if (cmd->type == B2_CMD_MOVE_FORWARD)
        {
            request[0] = goal_speed;
            request[1] = goal_speed;
        }
        else if (cmd->type == B2_CMD_MOVE_BACKWARD)
        {
            request[0] = -goal_speed;
            request[1] = -goal_speed;
        }
        else if (cmd->type == B2_CMD_TURN_LEFT)
        {
            request[0] = -goal_speed;
            request[1] = goal_speed;
        }
        else
        {
            request[0] = goal_speed;
            request[1] = -goal_speed;
        }
        s_goal_ticks = (cmd->type <= B2_CMD_MOVE_BACKWARD) ?
            b2_mm_to_ticks(cmd->distance_mm, s_cfg->ticks_per_wheel_rev,
                           s_cfg->wheel_circumference_mm) :
            b2_turn_to_ticks(cmd->turn_deg, s_cfg->wheel_track_mm,
                             s_cfg->ticks_per_wheel_rev,
                             s_cfg->wheel_circumference_mm);
        if (s_goal_ticks < B2_MIN_TICKS) { s_goal_ticks = B2_MIN_TICKS; }
        s_status.state = B2_EXECUTE_GOAL;
    }
    s_requested[0] = (int16_t)request[0];   /* |request| <= 511 */
    s_requested[1] = (int16_t)request[1];
    /* A wheel held at 0 (steering) has old encoder edges: give it the full
     * stall window again when it restarts, or it reads as stalled at once. */
    for (uint32_t i = 0U; i < 2U; ++i)
    {
        if ((previous[i] == 0) && (s_requested[i] != 0))
        {
            s_wheel_started_us[i] = now;
        }
        /* Anti-windup: the integral belongs to the old setpoint. When S3
         * swaps a wheel between inner (30 / -40) and outer (70), the error
         * stored while the slow wheel stalled pushed the new 70 mm/s
         * request to 100-127 mm/s (2026-11-17 telemetry) and widened every
         * zig-zag. A steady request (straight line, hump) keeps it. */
        if (((previous[i] < 0) != (s_requested[i] < 0)) ||
            (b2_abs32((int32_t)s_requested[i] - previous[i]) >=
             B2_INTEGRAL_RESET_MM_S))
        {
            s_integral[i] = 0;
        }
    }
    s_own.was_moving = 1U;
}
void b2_poll(void)
{
    if (s_shared.ready == 0U) { return; }
    const uint32_t now = app_time_us_32();
    uint32_t ticks[2];
    uint32_t edges_at_us[2];
    uint8_t generation_at_mailbox;
    b2_cmd_packed_t cmd;
    bool has_cmd;
    bool brake_event;
    UINT irq = app_lock(&s_lock);
    ticks[0] = s_irq_count[0];
    ticks[1] = s_irq_count[1];
    edges_at_us[0] = s_last_edge_us[0];
    edges_at_us[1] = s_last_edge_us[1];
    generation_at_mailbox = s_command_generation;
    has_cmd = (s_shared.pending != 0U);
    cmd = s_mailbox;
    s_shared.pending = 0U;
    app_unlock(&s_lock, irq);
    if (s_own.poll_started == 0U)
    {
        s_own.poll_started = 1U;
        s_last_poll_us = now;
        s_prev_ticks[0] = ticks[0];
        s_prev_ticks[1] = ticks[1];
        return;
    }
    const uint32_t elapsed_ms = (now - s_last_poll_us) / 1000U;
    s_last_poll_us = now;
    /* INTEGRATION FIX: the loop really runs every 20 ms (tk_dly_tsk(10) on
     * the 10 ms kernel tick), so the old 15 ms limit counted every loop as
     * a miss. */
    if ((elapsed_ms > 20U) && (s_status.deadline_misses < UINT16_MAX))
    {
        ++s_status.deadline_misses;
    }
    if (elapsed_ms == 0U) { return; }
    const uint32_t deltas[2] = {ticks[0] - s_prev_ticks[0],
                                 ticks[1] - s_prev_ticks[1]};
    s_prev_ticks[0] = ticks[0];
    s_prev_ticks[1] = ticks[1];
    irq = app_lock(&s_lock);
    s_status.left_encoder_edges = ticks[0];
    s_status.right_encoder_edges = ticks[1];
    /* Sign is inferred from the commanded direction (single-channel encoders). */
    for (uint32_t i = 0U; i < 2U; ++i)
    {
        const int32_t direction = (s_requested[i] < 0) ? -1 : 1;
        const int32_t raw = direction * b2_speed_mm_s(deltas[i], elapsed_ms,
            s_cfg->ticks_per_wheel_rev, s_cfg->wheel_circumference_mm);
        s_signed_encoder_ticks[i] += direction * (int32_t)deltas[i];
        const int64_t total_mm = ((int64_t)s_signed_encoder_ticks[i] *
            s_cfg->wheel_circumference_mm) / s_cfg->ticks_per_wheel_rev;
        const int16_t total = (total_mm > INT16_MAX) ? INT16_MAX :
                              (total_mm < INT16_MIN) ? INT16_MIN :
                              (int16_t)total_mm;
        if (i == 0U) { s_status.left_total_mm = total; }
        else         { s_status.right_total_mm = total; }
        b2_set_wheel_speed_locked(i,
            ((b2_wheel_speed_locked(i) * 3) + raw) / 4);
    }
    s_status.last_update_us = now;
    const bool braking = (s_status.brake_latched != 0U);
    /* CR1: was read and cleared without the lock. */
    brake_event = (s_shared.brake_event_pending != 0U);
    s_shared.brake_event_pending = 0U;
    app_unlock(&s_lock, irq);
    if (brake_event)
    {
        b2_queue_event(B2_EVENT_BRAKED, B2_FAULT_NONE);
    }
    if (braking)
    {
        b2_deliver_event();
        return;
    }
    if (has_cmd)
    {
        irq = app_lock(&s_lock);
        if ((generation_at_mailbox == s_command_generation) &&
            (s_status.brake_latched == 0U) &&
            ((s_status.state == B2_READY) ||
             (s_status.state == B2_TRACK_SPEED)))
        {
            b2_start_command(&cmd, ticks[0], ticks[1], now);
        }
        app_unlock(&s_lock, irq);
    }
    if ((s_status.state == B2_TRACK_SPEED) ||
        (s_status.state == B2_EXECUTE_GOAL))
    {
        if (s_status.state == B2_EXECUTE_GOAL)
        {
            if ((now - s_goal_started_us) >
                ((uint32_t)s_active.timeout_10ms *
                 (B2_TIMEOUT_UNIT_MS * 1000U)))
            {
                b2_fault_now(B2_FAULT_GOAL_TIMEOUT);
                b2_deliver_event();
                return;
            }
            bool reached[2];
            reached[0] = ((ticks[0] - s_start_ticks[0]) >= s_goal_ticks);
            reached[1] = ((ticks[1] - s_start_ticks[1]) >= s_goal_ticks);
            if (reached[0]) { s_requested[0] = 0; }
            if (reached[1]) { s_requested[1] = 0; }
            if (reached[0] && reached[1])
            {
                irq = app_lock(&s_lock);
                ++s_status.goals_done;
                app_unlock(&s_lock, irq);
                b2_stop();
                b2_queue_event(B2_EVENT_GOAL_DONE, B2_FAULT_NONE);
                b2_deliver_event();
                return;
            }
        }
        int32_t pwm_cmd[2];
        int32_t actual[2];
        irq = app_lock(&s_lock);
        actual[0] = b2_wheel_speed_locked(0U);
        actual[1] = b2_wheel_speed_locked(1U);
        app_unlock(&s_lock, irq);
        for (uint32_t i = 0U; i < 2U; ++i)
        {
            const int32_t request = s_requested[i];
            if (request == 0)
            {
                pwm_cmd[i] = 0;
                s_integral[i] = 0;
                continue;
            }
            const int32_t error = b2_abs32(request) - b2_abs32(actual[i]);
            /* +-8000 * ki 8/256 = +-250 permil: room for a slope, but no
             * longer enough to drive the motor on its own (was +-20000,
             * = +-625 of the 650 permil maximum). */
            s_integral[i] = (int16_t)b2_clamp((int32_t)s_integral[i] + error,
                                              -B2_INTEGRAL_LIMIT,
                                              B2_INTEGRAL_LIMIT);
            const int32_t ff = ((int32_t)s_cfg->max_pwm_permil *
                                b2_abs32(request)) / s_cfg->max_speed_mm_s;
            const int32_t p = ((int32_t)s_cfg->kp_q8 * error) / 256;
            const int32_t integral =
                ((int32_t)s_cfg->ki_q8 * s_integral[i]) / 256;
            const int32_t duty = b2_clamp(ff + p + integral,
                                             B2_MIN_OUTPUT, s_cfg->max_pwm_permil);
            pwm_cmd[i] = (request < 0) ? -duty : duty;
            /* INTEGRATION FIX: an encoder edge can land between reading `now`
             * and taking the lock, making the edge newer than `now`; the
             * unsigned subtraction then wraps to a huge age and a false
             * ENCODER_STALL.  Frequent at ~450 edges/s.  CR1: 32-bit times,
             * so "newer" is a wrap-safe signed compare. */
            const uint32_t age = b2_time_after(edges_at_us[i], now) ? 0U :
                                 (now - edges_at_us[i]);
            const uint32_t stall_us = (uint32_t)s_cfg->encoder_stall_ms * 1000U;
            if (((s_status.state == B2_EXECUTE_GOAL) ||
                 (s_status.state == B2_TRACK_SPEED)) &&
                (b2_abs32(request) >= 40) &&
                (age > stall_us) &&
                ((now - s_wheel_started_us[i]) > stall_us))
            {
                b2_fault_now(B2_FAULT_ENCODER_STALL);
                b2_deliver_event();
                return;
            }
        }
        irq = app_lock(&s_lock);
        /* E-STOP can arrive before the task writes its next PWM update. */
        if ((s_status.brake_latched == 0U) &&
            ((s_status.state == B2_TRACK_SPEED) ||
             (s_status.state == B2_EXECUTE_GOAL)))
        {
            b2_write_wheels_locked(pwm_cmd[0], pwm_cmd[1]);
        }
        app_unlock(&s_lock, irq);
    }
    /* CR1: stop_event_pending / s_last_stopped_check_us are also written by
     * b2_stop() on other tasks: check and update them under the lock. */
    bool stopped_event = false;
    irq = app_lock(&s_lock);
    if ((s_shared.stop_event_pending != 0U) &&
        ((now - s_last_stopped_check_us) >=
         (B2_STOPPED_STABLE_MS * 1000U)))
    {
        /* This indicates commanded stop & no new edges, not proof of rest. */
        if ((deltas[0] == 0U) && (deltas[1] == 0U))
        {
            s_shared.stop_event_pending = 0U;
            stopped_event = true;
        }
        else
        {
            s_last_stopped_check_us = now;
        }
    }
    app_unlock(&s_lock, irq);
    if (stopped_event)
    {
        b2_queue_event(B2_EVENT_STOPPED, B2_FAULT_NONE);
    }
    b2_deliver_event();
}
