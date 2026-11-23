/**
 * CR1 host test for subsystem 2 (motion.c): the REAL motion.c is compiled on
 * the laptop with the kernel and RP2040 hardware replaced by stubs, a fake
 * microsecond clock and a fake encoder interrupt. No Pico needed.
 *
 * From INF2004_Project/:
 *   gcc -std=gnu11 -Wall -Wextra testing/cr1/host_test/test_s2_motion.c \
 *       app_program/subsystem2/b2_math.c -o test_s2 && ./test_s2
 *
 * What it checks (CR1 evidence: no loss of functionality):
 *   - packed sizes of the shared structs and the mailbox
 *   - every command field survives packing, at the edges of its range
 *   - out-of-range requests are REFUSED, never truncated
 *   - a MOVE goal completes with the right distance and speed
 *   - 32-bit microsecond time wrap-around causes no false stall / timeout
 *   - encoder stall sets the fault BIT and latches the brake; reset clears it
 *   - emergency brake latches and blocks commands
 *   - odometry saturates at +-32767 mm instead of wrapping
 *   - every lock taken is released (no path leaves s_lock held)
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* ---------- stubs for app.h / app_hw.h / tkernel_adapter.h ---------- */
#define APP_H
#define APP_HW_H
#define B2_TKERNEL_ADAPTER_H
typedef unsigned int UINT;
typedef int ER;
typedef void (*FP)(void);
#define TRUE            (1)
#define E_OK            (0)
#define TA_HLNG         (0U)
#define S2_ENC_INTPRI   (2)
#define APP_CLK_SYS_HZ  (125000000u)
#define IO_BANK0_BASE   (0x40014000u)
#define PWM_BASE        (0x40050000u)
#define RESETS_RESET_PWM (1u << 14)
#define GPIO_MODE_IN    (0)
typedef struct { UINT intatr; FP inthdr; } T_DINT;

static uint32_t g_now_us;
static int g_lock_depth;
static int g_lock_depth_max;
static uint32_t g_cc[16];

static inline uint32_t app_time_us_32(void) { return g_now_us; }
typedef uint32_t app_lock_t;
#define APP_LOCK_INIT(l) ((void)(l))
static inline UINT app_lock(app_lock_t *l)
{
    (void)l;
    if (++g_lock_depth > g_lock_depth_max) { g_lock_depth_max = g_lock_depth; }
    return 0U;
}
static inline void app_unlock(app_lock_t *l, UINT s) { (void)l; (void)s; --g_lock_depth; }
static inline int pwm_set_cc(unsigned int g, uint32_t v) { g_cc[g & 15U] = v; return 0; }
static inline int pwm_set_pin(unsigned int g) { (void)g; return 0; }
static inline int pwm_set_wrap(unsigned int g, uint32_t w) { (void)g; (void)w; return 0; }
static inline int pwm_set_enabled(unsigned int g, int e) { (void)g; (void)e; return 0; }
static inline int gpio_set_pin(unsigned int g, int m) { (void)g; (void)m; return 0; }
static inline void app_release_reset(uint32_t m) { (void)m; }
static inline ER tk_def_int(UINT n, const T_DINT *d) { (void)n; (void)d; return E_OK; }
static inline void ClearInt(UINT n) { (void)n; }
static inline void EnableInt(UINT n, int p) { (void)n; (void)p; }
static inline uint32_t in_w(uint32_t a) { (void)a; return 0U; }
static inline void out_w(uint32_t a, uint32_t v) { (void)a; (void)v; }
static inline void set_w(uint32_t a, uint32_t v) { (void)a; (void)v; }

/* The code under test. */
#include "../../../app_program/subsystem2/motion.c"

/* ---------- test helpers ---------- */
static int g_failures;
static int g_checks;
#define CHECK(cond)                                                     \
    do {                                                                \
        ++g_checks;                                                     \
        if (!(cond)) {                                                  \
            printf("FAIL line %d: %s\n", __LINE__, #cond);              \
            ++g_failures;                                               \
        }                                                               \
    } while (0)

static const b2_config_t k_cfg = {
    .ticks_per_rev_left = 930U, .ticks_per_rev_right = 930U,
    .wheel_circumference_mm = 210U,
    .wheel_track_mm = 120U, .max_speed_mm_s = 300U, .max_pwm_permil = 650U,
    .encoder_stall_ms = 500U, .pwm_frequency_hz = 20000U,
    .kp_q8 = 128U, .ki_q8 = 8U,
    .kd_q8 = 0U, .straight_kp_q8 = 0U, .straight_trim_max_mm_s = 20U
};

/* Encoder simulation: each wheel produces edges at its own speed.
 * g_tpr_sim[] is the robot's REAL ticks per turn (may differ per wheel).
 * With g_model set, wheel speed follows the PWM through a simple motor:
 * deadband, gain (mm/s per permil, per wheel) and a 100 ms lag. */
static uint32_t g_edge_acc[2];  /* thousandths of a tick */
static int32_t g_wheel_mm_s[2];
static uint32_t g_tpr_sim[2] = {930U, 930U};
static int g_model;
static double g_motor_gain[2] = {0.52, 0.52};
static double g_model_v[2];
#define MODEL_DEADBAND_PERMIL (60.0)
#define MODEL_TAU_MS          (100.0)

static double model_pwm(uint32_t i)
{
    const uint32_t pin_a = (i == 0U) ? 8U : 10U;
    const double wrap = (double)s_pwm_wrap;
    if (g_cc[pin_a] != 0U) { return  1000.0 * g_cc[pin_a] / wrap; }
    if (g_cc[pin_a + 1U] != 0U) { return -1000.0 * g_cc[pin_a + 1U] / wrap; }
    return 0.0;
}

static void sim_ms(uint32_t ms)
{
    for (uint32_t t = 0U; t < ms; ++t)
    {
        g_now_us += 1000U;
        for (uint32_t i = 0U; i < 2U; ++i)
        {
            if (g_model)
            {
                const double pwm = model_pwm(i);
                const double mag = (pwm < 0.0) ? -pwm : pwm;
                double target = (mag > MODEL_DEADBAND_PERMIL) ?
                    g_motor_gain[i] * (mag - MODEL_DEADBAND_PERMIL) : 0.0;
                if (pwm < 0.0) { target = -target; }
                g_model_v[i] += (target - g_model_v[i]) / MODEL_TAU_MS;
                g_wheel_mm_s[i] = (int32_t)g_model_v[i];
            }
            /* A wheel only turns while S2 drives its PWM pins (A/B). */
            const uint32_t pin_a = (i == 0U) ? 8U : 10U;
            const bool driven = (g_cc[pin_a] != 0U) || (g_cc[pin_a + 1U] != 0U);
            int32_t v = (g_wheel_mm_s[i] < 0) ? -g_wheel_mm_s[i] : g_wheel_mm_s[i];
            if (!driven && !g_model) { v = 0; }
            g_edge_acc[i] += ((uint32_t)v * g_tpr_sim[i] * 1000U) / (210U * 1000U);
            while (g_edge_acc[i] >= 1000U)
            {
                g_edge_acc[i] -= 1000U;
                b2_encoder_gpio_irq((i == 0U) ? 6U : 7U, 0x8U);
            }
        }
        if ((t % 10U) == 9U) { b2_poll(); }
    }
}

static void fresh_start_cfg(uint32_t start_us, const b2_config_t *p_cfg);
static void fresh_start(uint32_t start_us)
{
    fresh_start_cfg(start_us, &k_cfg);
}
static void fresh_start_cfg(uint32_t start_us, const b2_config_t *p_cfg)
{
    g_now_us = start_us;
    g_wheel_mm_s[0] = 0;
    g_wheel_mm_s[1] = 0;
    g_edge_acc[0] = 0U;
    g_edge_acc[1] = 0U;
    g_model = 0;
    g_model_v[0] = 0.0;
    g_model_v[1] = 0.0;
    g_tpr_sim[0] = 930U;
    g_tpr_sim[1] = 930U;
    CHECK(b2_init(p_cfg, NULL, NULL));
    sim_ms(20U);
}

static b2_command_t speed_cmd(int16_t l, int16_t r)
{
    b2_command_t c = {0};
    c.type = B2_CMD_SPEED;
    c.left_mm_s = l;
    c.right_mm_s = r;
    return c;
}

/* ---------- tests ---------- */
static void test_sizes(void)
{
    CHECK(sizeof(b2_status_t) == 28U);      /* was 48 */
    CHECK(sizeof(b2_cmd_packed_t) == 8U);   /* mailbox, was 16 */
    CHECK(sizeof(b2_event_t) == 1U);        /* was 2 */
    CHECK(sizeof(b2_goal_t) == 8U);         /* was 24 */
}

static void test_pack_roundtrip(void)
{
    for (int32_t v = -511; v <= 511; ++v)
    {
        CHECK(b2_speed_value(b2_speed_field(v)) == v);
    }
    CHECK(b2_speed_value(b2_speed_field(-600)) == -512);  /* saturates */
    CHECK(b2_speed_value(b2_speed_field(600)) == 511);

    b2_command_t c = {0};
    c.type = B2_CMD_TURN_RIGHT;
    c.left_mm_s = -300;
    c.right_mm_s = 300;
    c.goal_speed_mm_s = 511U;
    c.distance_mm = B2_DISTANCE_MAX_MM;
    c.turn_degrees = B2_TURN_MAX_DEG;
    c.timeout_ms = B2_TIMEOUT_MAX_MS;
    b2_cmd_packed_t p = b2_pack_command(&c);
    CHECK(p.type == B2_CMD_TURN_RIGHT);
    CHECK(b2_speed_value(p.left_ob) == -300);
    CHECK(b2_speed_value(p.right_ob) == 300);
    CHECK(p.goal_speed == 511U);
    CHECK(p.distance_mm == 8191U);
    CHECK(p.turn_deg == 360U);
    CHECK(p.timeout_10ms == 1023U);

    c.timeout_ms = 1U;          /* rounds UP: never shorter than asked */
    CHECK(b2_pack_command(&c).timeout_10ms == 1U);
    c.timeout_ms = 6001U;
    CHECK(b2_pack_command(&c).timeout_10ms == 601U);

    b2_event_t e = {0};
    e.type = B2_EVENT_FAULT;
    e.fault = B2_FAULT_EVENT_LOSS;
    CHECK((e.type == B2_EVENT_FAULT) && (e.fault == B2_FAULT_EVENT_LOSS));
}

static void test_validation(void)
{
    fresh_start(1000000U);
    b2_command_t c = speed_cmd(10000, 0);
    CHECK(!b2_submit_command(&c));                  /* S2-U04 case */
    c = speed_cmd(301, 0);
    CHECK(!b2_submit_command(&c));                  /* > max_speed */

    b2_command_t g = {0};
    g.type = B2_CMD_MOVE_FORWARD;
    g.goal_speed_mm_s = 100U;
    g.timeout_ms = 5000U;
    g.distance_mm = B2_DISTANCE_MAX_MM + 1U;
    CHECK(!b2_submit_command(&g));                  /* would not fit 13 bits */
    g.distance_mm = 100U;
    g.timeout_ms = B2_TIMEOUT_MAX_MS + 1U;
    CHECK(!b2_submit_command(&g));                  /* would not fit 10 bits */
    g.timeout_ms = 0U;
    CHECK(!b2_submit_command(&g));
    g.type = B2_CMD_TURN_LEFT;
    g.timeout_ms = 5000U;
    g.turn_degrees = 361U;
    CHECK(!b2_submit_command(&g));
    g.turn_degrees = 90U;
    CHECK(b2_submit_command(&g));                   /* valid one accepted */
    CHECK(!b2_submit_command(NULL));
    CHECK(g_lock_depth == 0);
}

static void test_move_goal(void)
{
    fresh_start(2000000U);
    b2_command_t g = {0};
    b2_status_t st;
    g.type = B2_CMD_MOVE_FORWARD;
    g.distance_mm = 210U;           /* exactly one wheel turn = 930 ticks */
    g.goal_speed_mm_s = 100U;
    g.timeout_ms = 6000U;
    CHECK(b2_submit_command(&g));
    g_wheel_mm_s[0] = 100;
    g_wheel_mm_s[1] = 100;
    sim_ms(500U);
    b2_get_status(&st);
    CHECK(st.state == B2_EXECUTE_GOAL);
    CHECK((b2_speed_value(st.left_speed_ob) >= 90) &&
          (b2_speed_value(st.left_speed_ob) <= 110));
    CHECK(st.left_pwm_permil > 0U);
    sim_ms(2000U);
    g_wheel_mm_s[0] = 0;
    g_wheel_mm_s[1] = 0;
    sim_ms(50U);
    b2_get_status(&st);
    CHECK(st.goals_done == 1U);
    CHECK(st.state == B2_READY);
    CHECK((st.left_total_mm >= 205) && (st.left_total_mm <= 240));
    CHECK(st.fault_mask == 0U);
    CHECK(g_lock_depth == 0);
}

static void test_time_wrap(void)
{
    /* Start 0.3 s before the 32-bit microsecond counter wraps (71.6 min). */
    fresh_start(0xFFFFFFFFU - 300000U);
    b2_command_t c = speed_cmd(100, 100);
    b2_status_t st;
    CHECK(b2_submit_command(&c));
    g_wheel_mm_s[0] = 100;
    g_wheel_mm_s[1] = 100;
    sim_ms(1500U);                  /* crosses the wrap while driving */
    b2_get_status(&st);
    CHECK(g_now_us < 2000000U);     /* the clock really wrapped */
    CHECK(st.state == B2_TRACK_SPEED);
    CHECK(st.fault_mask == 0U);     /* no false stall */
    CHECK(st.brake_latched == 0U);

    /* A goal started just before the wrap must not time out after it. */
    b2_stop();
    sim_ms(300U);
    g_now_us = 0xFFFFFFFFU - 100000U;
    b2_command_t g = {0};
    g.type = B2_CMD_MOVE_FORWARD;
    g.distance_mm = 105U;
    g.goal_speed_mm_s = 100U;
    g.timeout_ms = 3000U;
    CHECK(b2_submit_command(&g));
    sim_ms(1500U);
    g_wheel_mm_s[0] = 0;
    g_wheel_mm_s[1] = 0;
    sim_ms(50U);
    b2_get_status(&st);
    CHECK(st.goals_done == 1U);
    CHECK(st.fault_mask == 0U);
    CHECK(g_lock_depth == 0);
}

static void test_stall_and_reset(void)
{
    fresh_start(3000000U);
    b2_command_t c = speed_cmd(100, 100);
    b2_status_t st;
    CHECK(b2_submit_command(&c));
    sim_ms(700U);                   /* no encoder edges at all */
    b2_get_status(&st);
    CHECK(st.state == B2_FAULT);
    CHECK(st.fault_mask == B2_FAULT_BIT(B2_FAULT_ENCODER_STALL));
    CHECK(st.brake_latched == 1U);
    CHECK((st.left_pwm_permil == 0U) && (st.right_pwm_permil == 0U));
    CHECK(!b2_submit_command(&c));
    sim_ms(300U);
    CHECK(b2_authorized_reset_while_stopped());
    b2_get_status(&st);
    CHECK((st.state == B2_READY) && (st.fault_mask == 0U) &&
          (st.brake_latched == 0U));
    CHECK(g_lock_depth == 0);
}

static void test_emergency_brake(void)
{
    fresh_start(4000000U);
    b2_command_t c = speed_cmd(100, 100);
    b2_status_t st;
    CHECK(b2_submit_command(&c));
    g_wheel_mm_s[0] = 100;
    g_wheel_mm_s[1] = 100;
    sim_ms(200U);
    b2_emergency_brake();           /* S4 lift / overturn path */
    b2_get_status(&st);
    CHECK((st.brake_latched == 1U) && (st.state == B2_BRAKED));
    CHECK((g_cc[8] == 0U) && (g_cc[9] == 0U) &&
          (g_cc[10] == 0U) && (g_cc[11] == 0U));
    CHECK(!b2_submit_command(&c));
    b2_stop();                      /* a plain STOP does not unlatch */
    b2_get_status(&st);
    CHECK(st.brake_latched == 1U);
    g_wheel_mm_s[0] = 0;
    g_wheel_mm_s[1] = 0;
    sim_ms(100U);
    CHECK(!b2_authorized_reset_while_stopped());  /* edges < 200 ms ago */
    sim_ms(300U);
    CHECK(b2_authorized_reset_while_stopped());
    CHECK(g_lock_depth == 0);
}

static void test_odometry_saturates(void)
{
    fresh_start(5000000U);
    b2_status_t st;
    s_signed_encoder_ticks[0] = 200000;     /* ~45 m of travel */
    s_signed_encoder_ticks[1] = -200000;
    sim_ms(20U);
    b2_get_status(&st);
    CHECK(st.left_total_mm == INT16_MAX);
    CHECK(st.right_total_mm == INT16_MIN);
}

/* ---------- B2 task-list additions (19 Nov) ---------- */
#include "../../../app_program/subsystem2/motion_cal.h"

/* The demo configuration: straight-line correction on, D off. */
static const b2_config_t k_cfg_demo = {
    .ticks_per_rev_left = 930U, .ticks_per_rev_right = 930U,
    .wheel_circumference_mm = 210U,
    .wheel_track_mm = 120U, .max_speed_mm_s = 300U, .max_pwm_permil = 650U,
    .encoder_stall_ms = 500U, .pwm_frequency_hz = 20000U,
    .kp_q8 = 128U, .ki_q8 = 8U,
    .kd_q8 = 0U, .straight_kp_q8 = 256U, .straight_trim_max_mm_s = 20U
};

static int32_t iabs(int32_t v) { return (v < 0) ? -v : v; }

/* Calibration header: S5's constants are derived from it and must not
 * change while the calibration values are unchanged. */
static void test_calibration_header(void)
{
    CHECK(B2_CAL_TICKS_PER_REV_MEAN == 930U);
    CHECK(B2_CAL_TICKS_PER_M == 4429U);             /* S5 had 4429 */
    CHECK(B2_CAL_TURN_TICKS_PER_DEG_X10 == 43U);    /* 20 Nov: track measured 110 mm (was 120 -> 46) */
    CHECK(b2_turn_to_mm(90U, 120U) == 94U);         /* pi*120/4 = 94.2 */
    CHECK(b2_turn_to_ticks(90U, 120U, 930U, 210U) ==
          b2_mm_to_ticks(94U, 930U, 210U));
    /* config validation: zero ticks or a trim over 511 mm/s refused */
    b2_config_t bad = k_cfg_demo;
    bad.ticks_per_rev_right = 0U;
    CHECK(!b2_init(&bad, NULL, NULL));
    bad = k_cfg_demo;
    bad.straight_trim_max_mm_s = 512U;
    CHECK(!b2_init(&bad, NULL, NULL));
}

/* Drive straight for `ms` with a 10 % weaker right motor, re-sending the
 * same speed every loop like S3/S5. Returns left-minus-right ticks. */
static int32_t run_straight(const b2_config_t *p_cfg, int16_t speed,
                            uint32_t ms, int32_t *p_speed_l, int32_t *p_speed_r)
{
    b2_status_t st;
    fresh_start_cfg(6000000U, p_cfg);
    g_model = 1;
    g_motor_gain[0] = 0.52;
    g_motor_gain[1] = 0.52 * 0.9;
    const uint32_t start[2] = {s_irq_count[0], s_irq_count[1]};
    b2_command_t c = speed_cmd(speed, speed);
    for (uint32_t t = 0U; t < ms; t += 20U)
    {
        CHECK(b2_submit_command(&c));
        sim_ms(20U);
    }
    b2_get_status(&st);
    *p_speed_l = b2_speed_value(st.left_speed_ob);
    *p_speed_r = b2_speed_value(st.right_speed_ob);
    CHECK(st.fault_mask == 0U);
    b2_stop();
    sim_ms(300U);
    g_model = 0;
    g_motor_gain[1] = 0.52;
    return (int32_t)(s_irq_count[0] - start[0]) -
           (int32_t)(s_irq_count[1] - start[1]);
}

static void test_straight_line(void)
{
    int32_t vl, vr;
    const int32_t off = run_straight(&k_cfg, 100, 5000U, &vl, &vr);
    const int32_t on = run_straight(&k_cfg_demo, 100, 5000U, &vl, &vr);
    printf("  straight 5 s @100 mm/s, right motor 10%% weak: L-R = %d ticks "
           "(%.1f mm) without correction, %d ticks (%.1f mm) with; "
           "end speeds %d/%d mm/s\n", off, off * 210.0 / 930.0, on,
           on * 210.0 / 930.0, vl, vr);
    CHECK(iabs(on) <= 5);               /* within ~1 mm */
    CHECK(iabs(on) < iabs(off));
    CHECK((vl >= 90) && (vl <= 110) && (vr >= 90) && (vr <= 110));
    /* Also straight in reverse and at S3's line speed. */
    CHECK(iabs(run_straight(&k_cfg_demo, -100, 3000U, &vl, &vr)) <= 5);
    CHECK(iabs(run_straight(&k_cfg_demo, 70, 3000U, &vl, &vr)) <= 5);
    CHECK(g_lock_depth == 0);
}

static void test_straight_trim_rules(void)
{
    fresh_start_cfg(7000000U, &k_cfg_demo);
    uint32_t ticks[2];
    s_straight_ref[0] = 0U;
    s_straight_ref[1] = 0U;
    /* steering (different speeds): no correction */
    s_requested[0] = 70;
    s_requested[1] = 30;
    ticks[0] = 100U; ticks[1] = 0U;
    CHECK(b2_straight_trim(ticks) == 0);
    /* straight: the wheel ahead is slowed, clamped at 20 mm/s */
    s_requested[1] = 70;
    CHECK(b2_straight_trim(ticks) == 20);
    ticks[0] = 0U; ticks[1] = 10U;
    CHECK(b2_straight_trim(ticks) == -10);
    /* on-the-spot turn (opposite signs, same size): also equalised */
    s_requested[1] = -70;
    CHECK(b2_straight_trim(ticks) == -10);
    /* counter wrap between reference and now */
    s_straight_ref[0] = 0xFFFFFFF0U;
    s_straight_ref[1] = 0xFFFFFFF0U;
    ticks[0] = 0x00000005U; ticks[1] = 0x00000000U;
    CHECK(b2_straight_trim(ticks) == 5);
    s_requested[0] = 0;
    s_requested[1] = 0;

    /* Re-sending the same pair keeps the reference; a new pair resets it. */
    fresh_start_cfg(7500000U, &k_cfg_demo);
    g_wheel_mm_s[0] = 70;
    g_wheel_mm_s[1] = 70;
    b2_command_t c = speed_cmd(70, 70);
    CHECK(b2_submit_command(&c));
    sim_ms(200U);
    const uint32_t ref = s_straight_ref[0];
    CHECK(b2_submit_command(&c));
    sim_ms(100U);
    CHECK(s_straight_ref[0] == ref);
    c = speed_cmd(70, 30);
    CHECK(b2_submit_command(&c));
    sim_ms(20U);
    CHECK(s_straight_ref[0] != ref);
    b2_stop();
    g_wheel_mm_s[0] = 0;
    g_wheel_mm_s[1] = 0;
    sim_ms(300U);

    /* Per-wheel calibration: equal distance is NOT equal ticks. */
    b2_config_t cfg = k_cfg_demo;
    cfg.ticks_per_rev_left = 962U;
    cfg.ticks_per_rev_right = 896U;
    fresh_start_cfg(7800000U, &cfg);
    s_requested[0] = 70;
    s_requested[1] = 70;
    s_straight_ref[0] = 0U;
    s_straight_ref[1] = 0U;
    ticks[0] = 962U; ticks[1] = 896U;   /* one wheel turn each */
    CHECK(b2_straight_trim(ticks) == 0);
    s_requested[0] = 0;
    s_requested[1] = 0;
    CHECK(g_lock_depth == 0);
}

/* Wheels with different ticks per turn (2026-11-08: L 962, R 896). */
static void test_per_wheel_calibration(void)
{
    b2_config_t cfg = k_cfg_demo;
    cfg.ticks_per_rev_left = 962U;
    cfg.ticks_per_rev_right = 896U;
    fresh_start_cfg(8000000U, &cfg);
    g_tpr_sim[0] = 962U;
    g_tpr_sim[1] = 896U;
    b2_status_t st;
    const uint32_t start[2] = {s_irq_count[0], s_irq_count[1]};
    b2_command_t g = {0};
    g.type = B2_CMD_MOVE_FORWARD;
    g.distance_mm = 210U;           /* one wheel turn */
    g.goal_speed_mm_s = 100U;
    g.timeout_ms = 6000U;
    CHECK(s_goal_ticks[0] == 0U);
    CHECK(b2_submit_command(&g));
    g_wheel_mm_s[0] = 100;
    g_wheel_mm_s[1] = 100;
    sim_ms(20U);
    CHECK((s_goal_ticks[0] == 962U) && (s_goal_ticks[1] == 896U));
    sim_ms(2500U);
    g_wheel_mm_s[0] = 0;
    g_wheel_mm_s[1] = 0;
    sim_ms(50U);
    b2_get_status(&st);
    CHECK(st.goals_done == 1U);
    CHECK((s_irq_count[0] - start[0]) >= 962U);
    CHECK((s_irq_count[1] - start[1]) >= 896U);
    /* both wheels report the same distance despite different tick counts */
    CHECK((st.left_total_mm >= 205) && (st.left_total_mm <= 220));
    CHECK((st.right_total_mm >= 205) && (st.right_total_mm <= 220));
    CHECK(iabs(st.left_total_mm - st.right_total_mm) <= 3);
    CHECK(g_lock_depth == 0);
}

/* Step 0 -> 150 mm/s on the motor model; returns the peak speed. */
static int32_t step_peak(uint16_t kd, int32_t *p_final)
{
    b2_config_t cfg = k_cfg_demo;
    cfg.kd_q8 = kd;
    b2_status_t st;
    int32_t peak = 0;
    fresh_start_cfg(9000000U, &cfg);
    g_model = 1;
    b2_command_t c = speed_cmd(150, 150);
    CHECK(b2_submit_command(&c));
    for (uint32_t t = 0U; t < 3000U; t += 20U)
    {
        sim_ms(20U);
        b2_get_status(&st);
        if (b2_speed_value(st.left_speed_ob) > peak)
        {
            peak = b2_speed_value(st.left_speed_ob);
        }
    }
    *p_final = b2_speed_value(st.left_speed_ob);
    b2_stop();
    sim_ms(300U);
    g_model = 0;
    return peak;
}

static void test_pid_d_term(void)
{
    int32_t final0, final_d;
    const int32_t peak0 = step_peak(0U, &final0);
    const int32_t peak_d = step_peak(512U, &final_d);
    printf("  step 0->150 mm/s on the motor model: kd=0 peak %d final %d; "
           "kd=2.0 peak %d final %d\n", peak0, final0, peak_d, final_d);
    CHECK((final0 >= 140) && (final0 <= 160));
    CHECK((final_d >= 140) && (final_d <= 160));   /* D does not destabilise */
    /* One encoder tick per 20 ms loop is ~11 mm/s, so the measured speed
     * is coarse and D changes the peak by only a few mm/s either way
     * (kd 0..8.0 swept: 161..164). Hence kd = 0 in the demo config. */
    CHECK(iabs(peak_d - peak0) <= 5);
    CHECK(g_lock_depth == 0);
}

static void test_api_wrappers(void)
{
    b2_goal_t goal;
    fresh_start_cfg(10000000U, &k_cfg_demo);
    CHECK(!b2_move_forward(0U, NULL));              /* nothing to do */
    CHECK(!b2_move_forward(1200U, NULL));           /* > 10.23 s at 120 mm/s */
    CHECK(!b2_turn_left(361U, NULL));
    CHECK(!b2_turn_right(0U, NULL));
    CHECK(b2_move_forward(1100U, NULL));            /* fits: 9.2 s + 1 s */
    CHECK(s_mailbox.type == B2_CMD_MOVE_FORWARD);
    CHECK(s_mailbox.distance_mm == 1100U);
    CHECK(s_mailbox.goal_speed == B2_API_MOVE_SPEED_MM_S);
    CHECK(s_mailbox.timeout_10ms == 1023U);         /* capped at 10.23 s */
    b2_stop();
    sim_ms(300U);

    /* A turn goal, followed with b2_goal_check, on the motor model with
     * a weak right motor: both wheels finish together. */
    g_model = 1;
    g_motor_gain[1] = 0.52 * 0.9;
    const uint32_t start[2] = {s_irq_count[0], s_irq_count[1]};
    CHECK(b2_turn_left(90U, &goal));
    CHECK(s_mailbox.type == B2_CMD_TURN_LEFT);
    CHECK(s_mailbox.turn_deg == 90U);
    CHECK(s_mailbox.goal_speed == B2_API_TURN_SPEED_MM_S);
    CHECK(s_mailbox.timeout_10ms == 335U);          /* 94 mm @80 -> 2x1.175+1 s */
    CHECK(!b2_move_backward(100U, NULL));           /* one goal at a time */
    b2_goal_result_t r = B2_GOAL_RUNNING;
    for (uint32_t t = 0U; (t < 4000U) && (r == B2_GOAL_RUNNING); t += 20U)
    {
        sim_ms(20U);
        r = b2_goal_check(&goal);
    }
    CHECK(r == B2_GOAL_DONE);
    const int32_t dl = (int32_t)(s_irq_count[0] - start[0]);
    const int32_t dr = (int32_t)(s_irq_count[1] - start[1]);
    printf("  b2_turn_left(90): L %d ticks, R %d ticks (goal %u each)\n",
           dl, dr, (unsigned)s_goal_ticks[0]);
    CHECK(dl >= (int32_t)s_goal_ticks[0]);
    CHECK(dr >= (int32_t)s_goal_ticks[1]);
    CHECK(iabs(dl - dr) <= 10);
    g_model = 0;
    g_motor_gain[1] = 0.52;
    sim_ms(300U);

    /* Refused while tracking a speed (not READY) or braked. */
    b2_command_t c = speed_cmd(100, 100);
    g_wheel_mm_s[0] = 100;
    g_wheel_mm_s[1] = 100;
    CHECK(b2_submit_command(&c));
    sim_ms(40U);
    CHECK(!b2_move_forward(100U, NULL));
    b2_emergency_brake();
    g_wheel_mm_s[0] = 0;
    g_wheel_mm_s[1] = 0;
    sim_ms(300U);
    CHECK(!b2_turn_right(90U, NULL));
    CHECK(b2_authorized_reset_while_stopped());
    CHECK(b2_move_backward(100U, NULL));
    b2_stop();
    sim_ms(300U);
    CHECK(g_lock_depth == 0);
}

int main(void)
{
    test_sizes();
    test_pack_roundtrip();
    test_validation();
    test_move_goal();
    test_time_wrap();
    test_stall_and_reset();
    test_emergency_brake();
    test_odometry_saturates();
    test_calibration_header();
    test_straight_line();
    test_straight_trim_rules();
    test_per_wheel_calibration();
    test_pid_d_term();
    test_api_wrappers();
    CHECK(g_lock_depth == 0);
    CHECK(g_lock_depth_max == 1);   /* s_lock is never taken twice */
    printf("S2 motion host test: %d checks, %d failed\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
