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
#define B2_GOAL_START_US     (100000ULL)

static b2_config_t s_cfg;
static b2_status_t s_status;
static b2_event_publish_fn s_publish;
static void *s_publish_ctx;
static volatile uint32_t s_irq_count[2];
static volatile uint64_t s_last_edge_us[2];
static bool s_ready;
static bool s_pending;
static b2_command_t s_mailbox;
static b2_command_t s_active;
static uint32_t s_goal_ticks;
static uint32_t s_start_ticks[2];
static uint32_t s_prev_ticks[2];
static int32_t s_requested[2];
static int32_t s_integral[2];
static uint64_t s_goal_started_us;
static uint64_t s_wheel_started_us[2];  /* last time this wheel went 0 -> moving */
static uint64_t s_last_poll_us;
static uint64_t s_last_stopped_check_us;
static uint32_t s_pwm_wrap;
static volatile uint32_t s_command_generation;
static int64_t s_signed_encoder_ticks[2];
static bool s_was_moving;
static bool s_stop_event_pending;
static bool s_brake_event_pending;
static b2_event_t s_pending_event;
static bool s_event_pending;

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
    s_status.left_pwm_permil = (uint16_t)b2_abs32(a);
    s_status.right_pwm_permil = (uint16_t)b2_abs32(b);
}
static void b2_zero_outputs_locked(void)
{
    b2_write_wheels_locked(0, 0);
    s_requested[0] = 0;
    s_requested[1] = 0;
    s_integral[0] = 0;
    s_integral[1] = 0;
}
static void b2_queue_event(b2_event_type_t type, b2_fault_t fault)
{
    s_pending_event.type = type;
    s_pending_event.fault = fault;
    s_event_pending = true;
}
static void b2_fault_now(b2_fault_t fault)
{
    const UINT irq = app_lock(&s_lock);
    b2_zero_outputs_locked();
    s_status.state = B2_FAULT;
    s_status.fault = fault;
    s_status.brake_latched = 1U;
    s_pending = false;
    ++s_command_generation;
    app_unlock(&s_lock, irq);
    b2_queue_event(B2_EVENT_FAULT, fault);
}
static void b2_deliver_event(void)
{
    if (s_event_pending)
    {
        const b2_event_t event = s_pending_event;
        s_event_pending = false;
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
    s_cfg = *config;
    s_publish = publish;
    s_publish_ctx = ctx;
    s_status.state = B2_READY;
    s_command_generation = 0U;
    s_signed_encoder_ticks[0] = 0LL;
    s_signed_encoder_ticks[1] = 0LL;
    s_irq_count[0] = 0U;
    s_irq_count[1] = 0U;
    s_last_edge_us[0] = 0ULL;
    s_last_edge_us[1] = 0ULL;
    s_ready = false;
    s_pending = false;
    s_event_pending = false;
    s_brake_event_pending = false;
    s_was_moving = false;
    s_stop_event_pending = false;
    s_last_poll_us = 0ULL;
    s_last_stopped_check_us = 0ULL;
    s_pwm_wrap = (APP_CLK_SYS_HZ / config->pwm_frequency_hz) - 1U;  /* PORT */
    if ((s_pwm_wrap < 100U) || (s_pwm_wrap > 65535U))
    {
        return false;
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
    s_ready = true;
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
            s_last_edge_us[i] = app_time_us_64();
        }
    }
}
bool b2_submit_command(const b2_command_t *cmd)
{
    if ((!s_ready) || (cmd == NULL)) { return false; }
    if (cmd->type > B2_CMD_TURN_RIGHT) { return false; }
    if (cmd->type == B2_CMD_SPEED)
    {
        if ((b2_abs32(cmd->left_mm_s) > s_cfg.max_speed_mm_s) ||
            (b2_abs32(cmd->right_mm_s) > s_cfg.max_speed_mm_s))
        { return false; }
    }
    else if ((cmd->goal_speed_mm_s == 0U) ||
             (cmd->goal_speed_mm_s > s_cfg.max_speed_mm_s) ||
             (cmd->timeout_ms == 0U) ||
             (((cmd->type == B2_CMD_MOVE_FORWARD) ||
               (cmd->type == B2_CMD_MOVE_BACKWARD)) &&
               (cmd->distance_mm == 0U)) ||
             (((cmd->type == B2_CMD_TURN_LEFT) ||
               (cmd->type == B2_CMD_TURN_RIGHT)) &&
               (cmd->turn_degrees == 0U)))
    { return false; }
    const UINT irq = app_lock(&s_lock);
    const bool allowed = (s_status.brake_latched == 0U) &&
        (s_status.state != B2_FAULT) &&
        ((s_status.state == B2_READY) ||
         ((s_status.state == B2_TRACK_SPEED) && (cmd->type == B2_CMD_SPEED))) &&
        (!s_pending || (cmd->type == B2_CMD_SPEED &&
                        s_mailbox.type == B2_CMD_SPEED));
    if (allowed)
    {
        s_mailbox = *cmd;
        s_pending = true;
    }
    app_unlock(&s_lock, irq);
    return allowed;
}
void b2_stop(void)
{
    const UINT irq = app_lock(&s_lock);
    if (s_ready && !s_status.brake_latched)
    {
        b2_zero_outputs_locked();
        s_status.state = B2_READY;
        s_pending = false;
        ++s_command_generation;
        s_stop_event_pending = true;
        s_last_stopped_check_us = app_time_us_64();
    }
    app_unlock(&s_lock, irq);
}
void b2_emergency_brake(void)
{
    if (!s_ready) { return; }
    const UINT irq = app_lock(&s_lock);
    b2_zero_outputs_locked();
    s_status.brake_latched = 1U;
    s_status.state = B2_BRAKED;
    s_pending = false;
    ++s_command_generation;
    s_brake_event_pending = true;
    app_unlock(&s_lock, irq);
}
bool b2_authorized_reset_while_stopped(void)
{
    if (!s_ready) { return false; }
    const UINT irq = app_lock(&s_lock);
    /* Encoder silence supports but cannot prove physical standstill. */
    const uint64_t now = app_time_us_64();
    if (((s_last_edge_us[0] != 0ULL) &&
         ((now - s_last_edge_us[0]) < 200000ULL)) ||
        ((s_last_edge_us[1] != 0ULL) &&
         ((now - s_last_edge_us[1]) < 200000ULL)))
    {
        app_unlock(&s_lock, irq);
        return false;
    }
    /* Must not release the latch merely due to a new START command. */
    b2_zero_outputs_locked();
    s_status.brake_latched = 0U;
    s_status.fault = B2_FAULT_NONE;
    s_status.state = B2_READY;
    s_pending = false;
    ++s_command_generation;
    s_stop_event_pending = false;
    s_brake_event_pending = false;
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
    goal->started_us = app_time_us_64();
    goal->seen_running = false;
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
        goal->seen_running = true;
        return B2_GOAL_RUNNING;
    }
    /* Still READY shortly after the start: waiting for the next b2_poll(). */
    if ((st.state == B2_READY) && !goal->seen_running &&
        ((app_time_us_64() - goal->started_us) < B2_GOAL_START_US))
    {
        return B2_GOAL_RUNNING;
    }
    return B2_GOAL_FAILED;
}
static void b2_start_command(const b2_command_t *cmd, uint32_t tick_l,
                             uint32_t tick_r, uint64_t now)
{
    const bool continuous_speed = (cmd->type == B2_CMD_SPEED) &&
                                  (s_status.state == B2_TRACK_SPEED);
    const int32_t previous[2] = {s_requested[0], s_requested[1]};
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
        s_requested[0] = cmd->left_mm_s;
        s_requested[1] = cmd->right_mm_s;
        s_status.state = B2_TRACK_SPEED;
    }
    else
    {
        if (cmd->type == B2_CMD_MOVE_FORWARD)
        {
            s_requested[0] = cmd->goal_speed_mm_s;
            s_requested[1] = cmd->goal_speed_mm_s;
        }
        else if (cmd->type == B2_CMD_MOVE_BACKWARD)
        {
            s_requested[0] = -(int32_t)cmd->goal_speed_mm_s;
            s_requested[1] = -(int32_t)cmd->goal_speed_mm_s;
        }
        else if (cmd->type == B2_CMD_TURN_LEFT)
        {
            s_requested[0] = -(int32_t)cmd->goal_speed_mm_s;
            s_requested[1] = cmd->goal_speed_mm_s;
        }
        else
        {
            s_requested[0] = cmd->goal_speed_mm_s;
            s_requested[1] = -(int32_t)cmd->goal_speed_mm_s;
        }
        s_goal_ticks = (cmd->type <= B2_CMD_MOVE_BACKWARD) ?
            b2_mm_to_ticks(cmd->distance_mm, s_cfg.ticks_per_wheel_rev,
                           s_cfg.wheel_circumference_mm) :
            b2_turn_to_ticks(cmd->turn_degrees, s_cfg.wheel_track_mm,
                             s_cfg.ticks_per_wheel_rev,
                             s_cfg.wheel_circumference_mm);
        if (s_goal_ticks < B2_MIN_TICKS) { s_goal_ticks = B2_MIN_TICKS; }
        s_status.state = B2_EXECUTE_GOAL;
    }
    /* A wheel held at 0 (steering) has old encoder edges: give it the full
     * stall window again when it restarts, or it reads as stalled at once. */
    for (uint32_t i = 0U; i < 2U; ++i)
    {
        if ((previous[i] == 0) && (s_requested[i] != 0))
        {
            s_wheel_started_us[i] = now;
        }
    }
    s_was_moving = true;
}
void b2_poll(void)
{
    if (!s_ready) { return; }
    const uint64_t now = app_time_us_64();
    uint32_t ticks[2];
    uint64_t edges_at_us[2];
    uint32_t generation_at_mailbox;
    b2_command_t cmd;
    bool has_cmd;
    UINT irq = app_lock(&s_lock);
    ticks[0] = s_irq_count[0];
    ticks[1] = s_irq_count[1];
    edges_at_us[0] = s_last_edge_us[0];
    edges_at_us[1] = s_last_edge_us[1];
    generation_at_mailbox = s_command_generation;
    has_cmd = s_pending;
    cmd = s_mailbox;
    s_pending = false;
    app_unlock(&s_lock, irq);
    if (s_last_poll_us == 0ULL)
    {
        s_last_poll_us = now;
        s_prev_ticks[0] = ticks[0];
        s_prev_ticks[1] = ticks[1];
        return;
    }
    const uint32_t elapsed_ms = (uint32_t)((now - s_last_poll_us) / 1000ULL);
    s_last_poll_us = now;
    if (elapsed_ms > 15U) { ++s_status.deadline_misses; }
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
            s_cfg.ticks_per_wheel_rev, s_cfg.wheel_circumference_mm);
        int32_t *total = (i == 0U) ? &s_status.left_total_mm :
                                      &s_status.right_total_mm;
        int32_t *speed = (i == 0U) ? &s_status.left_speed_mm_s :
                                      &s_status.right_speed_mm_s;
        s_signed_encoder_ticks[i] += (int64_t)direction * deltas[i];
        const int64_t total_mm = (s_signed_encoder_ticks[i] *
            s_cfg.wheel_circumference_mm) / s_cfg.ticks_per_wheel_rev;
        *total = (total_mm > INT32_MAX) ? INT32_MAX :
                 (total_mm < INT32_MIN) ? INT32_MIN : (int32_t)total_mm;
        *speed = ((*speed * 3) + raw) / 4;
    }
    s_status.last_update_us = now;
    const bool braking = (s_status.brake_latched != 0U);
    app_unlock(&s_lock, irq);
    if (s_brake_event_pending)
    {
        s_brake_event_pending = false;
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
                ((uint64_t)s_active.timeout_ms * 1000ULL))
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
        const int32_t actual[2] = {s_status.left_speed_mm_s,
                                    s_status.right_speed_mm_s};
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
            s_integral[i] = b2_clamp(s_integral[i] + error, -20000, 20000);
            const int32_t ff = ((int32_t)s_cfg.max_pwm_permil *
                                b2_abs32(request)) / s_cfg.max_speed_mm_s;
            const int32_t p = ((int32_t)s_cfg.kp_q8 * error) / 256;
            const int32_t integral =
                ((int32_t)s_cfg.ki_q8 * s_integral[i]) / 256;
            const int32_t duty = b2_clamp(ff + p + integral,
                                             B2_MIN_OUTPUT, s_cfg.max_pwm_permil);
            pwm_cmd[i] = (request < 0) ? -duty : duty;
            const uint64_t age = now - edges_at_us[i];
            if (((s_status.state == B2_EXECUTE_GOAL) ||
                 (s_status.state == B2_TRACK_SPEED)) &&
                (b2_abs32(request) >= 40) &&
                (age > ((uint64_t)s_cfg.encoder_stall_ms * 1000ULL)) &&
                ((now - s_wheel_started_us[i]) >
                  ((uint64_t)s_cfg.encoder_stall_ms * 1000ULL)))
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
    if (s_stop_event_pending &&
        ((now - s_last_stopped_check_us) >=
         ((uint64_t)B2_STOPPED_STABLE_MS * 1000ULL)))
    {
        /* This indicates commanded stop & no new edges, not proof of rest. */
        if ((deltas[0] == 0U) && (deltas[1] == 0U))
        {
            s_stop_event_pending = false;
            b2_queue_event(B2_EVENT_STOPPED, B2_FAULT_NONE);
        }
        else
        {
            s_last_stopped_check_us = now;
        }
    }
    b2_deliver_event();
}
