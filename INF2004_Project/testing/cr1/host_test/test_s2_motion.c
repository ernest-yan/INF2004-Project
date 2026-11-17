/**
 * CR1 host test for subsystem 2 (motion.c): the REAL motion.c is compiled on
 * the laptop with the kernel and RP2040 hardware replaced by stubs, a fake
 * microsecond clock and a fake encoder interrupt. No Pico needed.
 *
 * From INF2004_Project/:
 *   gcc -std=gnu11 -Wall -Wextra testing/cr1/host_test/test_s2_motion.c \
 *       app_program/subsystem2/math.c -o test_s2 && ./test_s2
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
    .ticks_per_wheel_rev = 930U, .wheel_circumference_mm = 210U,
    .wheel_track_mm = 120U, .max_speed_mm_s = 300U, .max_pwm_permil = 650U,
    .encoder_stall_ms = 500U, .pwm_frequency_hz = 20000U,
    .kp_q8 = 128U, .ki_q8 = 8U
};

/* Encoder simulation: each wheel produces edges at its own speed. */
static uint32_t g_edge_acc[2];  /* thousandths of a tick */
static int32_t g_wheel_mm_s[2];

static void sim_ms(uint32_t ms)
{
    for (uint32_t t = 0U; t < ms; ++t)
    {
        g_now_us += 1000U;
        for (uint32_t i = 0U; i < 2U; ++i)
        {
            /* A wheel only turns while S2 drives its PWM pins (A/B). */
            const uint32_t pin_a = (i == 0U) ? 8U : 10U;
            const bool driven = (g_cc[pin_a] != 0U) || (g_cc[pin_a + 1U] != 0U);
            int32_t v = (g_wheel_mm_s[i] < 0) ? -g_wheel_mm_s[i] : g_wheel_mm_s[i];
            if (!driven) { v = 0; }
            g_edge_acc[i] += ((uint32_t)v * 930U * 1000U) / (210U * 1000U);
            while (g_edge_acc[i] >= 1000U)
            {
                g_edge_acc[i] -= 1000U;
                b2_encoder_gpio_irq((i == 0U) ? 6U : 7U, 0x8U);
            }
        }
        if ((t % 10U) == 9U) { b2_poll(); }
    }
}

static void fresh_start(uint32_t start_us)
{
    g_now_us = start_us;
    g_wheel_mm_s[0] = 0;
    g_wheel_mm_s[1] = 0;
    g_edge_acc[0] = 0U;
    g_edge_acc[1] = 0U;
    CHECK(b2_init(&k_cfg, NULL, NULL));
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
    CHECK(g_lock_depth == 0);
    CHECK(g_lock_depth_max == 1);   /* s_lock is never taken twice */
    printf("S2 motion host test: %d checks, %d failed\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
