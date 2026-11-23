/**
 * S3 junction simulator (2026-11-23, no track left before the demo).
 *
 * Compiles the REAL line_barcode.c with APP_MISSION_ENABLE = 1: barcode
 * decoder, line following, junction detection and the barcode turn all run
 * unchanged.  Only the kernel, GPIO and S1/S2/S4/S5 are stubbed.  A simple
 * 2-wheel car drives a virtual course:
 *
 *            +y (forward exit)
 *             |
 *   -x  ------+------  +x       18 mm black tape, cross at (0,0)
 *   (left)    |      (right)
 *             |  |||| Code 39 "*X*" beside the line on the right
 *             |       (20 mm gap, as the course spec), 300 mm before
 *           start     the junction
 *
 * The car starts on the -y arm heading +y.  For each map action the test
 * checks which arm the car leaves the junction on.
 *
 * Build + run (from the repo):
 *   gcc -std=gnu11 -O2 -w -IINF2004_Project/app_program/subsystem3 \
 *       INF2004_Project/testing/cr1/host_test/s3_junction_sim.c -o s3sim -lm
 *   ./s3sim          (summary)      ./s3sim -v   (S3 console log too)
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <setjmp.h>
#include <math.h>

/* ---------- stubs: app.h / app_hw.h / test_report.h / kernel ---------- */
#define APP_H
#define APP_HW_H
#define TEST_REPORT_H
#define APP_TEST            0
#define APP_BARCODE_ONLY    0
#define APP_MISSION_ENABLE  1
#define TM_MQTT             0
#define IMPORT  extern
#define EXPORT
#define LOCAL   static
typedef int ID;
typedef int ER;
typedef int INT;
typedef unsigned int UINT;
typedef int BOOL;
typedef unsigned char UB;
typedef int TMO;
typedef unsigned int ATR;
typedef int SZ;
#define TRUE    (1)
#define FALSE   (0)
#define E_OK    (0)
#define TMO_FEVR (-1)
#define TMO_POL  (0)
#define TA_TFIFO (0U)
#define TA_TPRI  (0U)
#define TA_INHERIT (0U)
#define GPIO_MODE_IN (0)
#define TP_PRC1 (1)
#define TP_PRC2 (2)
typedef struct { ATR mtxatr; int ceilpri; void *exinf; } T_CMTX;
typedef struct { ATR mbfatr; SZ bufsz; INT maxmsz; void *exinf; } T_CMBF;

static uint32_t g_now_us;
static int g_pin[32];
static int g_verbose;
static volatile uint32_t *g_timerawl;

/* message buffer mbf_barcode */
static char g_q[16];
static int g_qn;

static void sim_advance_ms(int ms);

static inline uint32_t app_time_us_32(void) { return g_now_us; }
static inline ER tk_cre_mtx(const T_CMTX *c) { (void)c; return 1; }
static inline ER tk_loc_mtx(ID i, TMO t) { (void)i; (void)t; return E_OK; }
static inline ER tk_unl_mtx(ID i) { (void)i; return E_OK; }
static inline ER tk_cre_mbf(const T_CMBF *c) { (void)c; return 2; }
static inline ER tk_snd_mbf(ID i, const void *m, INT s, TMO t)
{
    (void)i; (void)s; (void)t;
    if (g_qn < (int)sizeof(g_q)) { g_q[g_qn++] = *(const char *)m; }
    return E_OK;
}
static inline ER tk_rcv_mbf(ID i, void *m, TMO t)
{
    (void)i; (void)t;
    if (g_qn == 0) { return -1; }
    *(char *)m = g_q[0];
    memmove(g_q, g_q + 1, (size_t)--g_qn);
    return 1;
}
static inline ER tk_dly_tsk(INT t) { sim_advance_ms(t); return E_OK; }
static char g_lastlog[160];
static int tm_printf(const UB *f, ...)
{
    va_list ap;
    va_start(ap, f);
    vsnprintf(g_lastlog, sizeof g_lastlog, (const char *)f, ap);
    va_end(ap);
    if (g_verbose) {
        extern double g_x, g_y, g_th;
        printf("      %7.3f s  axle (%6.1f,%6.1f) hdg %5.0f  %s", g_now_us / 1e6,
               g_x, g_y, g_th * 57.2958, g_lastlog);
    }
    return 0;
}
static inline int gpio_set_pin(unsigned int p, int m) { (void)p; (void)m; return 0; }
static inline int gpio_get_val(unsigned int p) { return g_pin[p & 31U]; }
static inline ID app_start_task(void (*f)(INT, void *), int pri, int prc, const char *n)
{ (void)f; (void)pri; (void)prc; (void)n; return 3; }

#include "line_barcode.c"

/* ---------- fake RP2040 timer word (line_barcode.c reads 0x40054028) ---------- */
#ifdef _WIN32
__declspec(dllimport) void *__stdcall VirtualAlloc(void *, size_t, unsigned long, unsigned long);
static void map_timer(void)
{
    void *p = VirtualAlloc((void *)0x40050000UL, 0x10000U, 0x3000UL /* RESERVE|COMMIT */, 0x04UL);
    if (p != (void *)0x40050000UL) { printf("cannot map the fake RP2040 timer page\n"); exit(2); }
    g_timerawl = (volatile uint32_t *)0x40054028UL;
}
#else
#include <sys/mman.h>
static void map_timer(void)
{
    void *p = mmap((void *)0x40054000UL, 4096U, PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    if (p != (void *)0x40054000UL) { printf("cannot map the fake RP2040 timer page\n"); exit(2); }
    g_timerawl = (volatile uint32_t *)0x40054028UL;
}
#endif

/* ---------- world ---------- */
#define TAPE_HALF   9.0     /* 18 mm tape */
#define ARM_LEN     450.0   /* each arm from the centre */
#define APPROACH    1000.0  /* approach arm length */
#define BC_X0       29.0    /* barcode strip: 20 mm gap from the tape edge */
#define BC_X1       79.0
#define BC_NARROW   3.0
#define BC_WIDE     9.0

static double g_spot;               /* each bar edge moves out this far (sensor threshold) */
static double g_lat_min, g_lat_max; /* GP16 sideways range over the barcode */
static double g_bar_a[40], g_bar_b[40];
static int g_nbars;
static double g_extra_tape[8][4];   /* extra rectangles x0,y0,x1,y1 */
static int g_nextra;

static uint16_t pattern_of(char c)
{
    for (unsigned i = 0U; i < sizeof(CODE39_TABLE) / sizeof(CODE39_TABLE[0]); i++) {
        if (CODE39_TABLE[i].character == c) { return CODE39_TABLE[i].pattern; }
    }
    return 0U;
}

static void place_barcode(char letter, double y_start)
{
    const uint16_t ch[3] = { pattern_of('*'), pattern_of(letter), pattern_of('*') };
    double y = y_start;
    int k = 0;
    g_nbars = 0;
    for (int c = 0; c < 3; c++) {
        for (int b = 8; b >= 0; b--, k++) {
            const double w = ((ch[c] >> b) & 1U) ? BC_WIDE : BC_NARROW;
            if ((k % 2) == 0) { g_bar_a[g_nbars] = y; g_bar_b[g_nbars] = y + w; g_nbars++; }
            y += w;
        }
        if (c < 2) { y += BC_NARROW; k = 0; }
    }
}

static int is_black(double x, double y)
{
    if (fabs(x) <= TAPE_HALF && y >= -APPROACH && y <= ARM_LEN) { return 1; }
    if (fabs(y) <= TAPE_HALF && fabs(x) <= ARM_LEN) { return 1; }
    for (int i = 0; i < g_nextra; i++) {
        if (x >= g_extra_tape[i][0] && x <= g_extra_tape[i][2] &&
            y >= g_extra_tape[i][1] && y <= g_extra_tape[i][3]) { return 1; }
    }
    if (x >= BC_X0 && x <= BC_X1) {
        for (int i = 0; i < g_nbars; i++) {
            if (y >= g_bar_a[i] - g_spot && y <= g_bar_b[i] + g_spot) { return 1; }
        }
    }
    return 0;
}

/* ---------- car ---------- */
typedef struct {
    double sensor_fwd;   /* sensor row ahead of the axle, mm (S3 assumes 60) */
    double left_lat;     /* GP2 left of GP4, mm */
    double right_lat;    /* GP16 right of GP4, mm */
    double enc_scale;    /* encoder edges per real mm vs calibration */
    double turn_gain;    /* real rotation / encoder rotation (19 Nov: ~1.7) */
    double accel;        /* wheel accel limit, mm/s^2 */
    double x0;           /* start lateral offset */
} car_cfg_t;

static car_cfg_t C;
/* decode stress: IR spot radius (bars look wider), wheel speed noise */
static double g_noise, g_gl = 1.0, g_gr = 1.0;
static int g_decode_only;
static uint32_t g_rng = 12345U;
static double frand(void) { g_rng = g_rng * 1103515245U + 12345U; return ((g_rng >> 8) & 0xFFFF) / 65535.0; }
double g_x, g_y, g_th;              /* axle centre, heading (rad) */
static double g_vl, g_vr, g_cl, g_cr;      /* actual / commanded wheel speed */
static double g_el, g_er;                  /* encoder edges (fractional) */
static int g_goal_active, g_goal_done;
static double g_goal_mm, g_goal_trav, g_goal_speed;
static double g_track = B2_CAL_WHEEL_TRACK_MM;
static const double g_tpm = (double)B2_CAL_TICKS_PER_REV_MEAN / B2_CAL_WHEEL_CIRCUMFERENCE_MM;

/* outcome */
static jmp_buf g_end;
static int g_junction_seen;
static int g_lost;
static char g_decoded;
static uint32_t g_t_end;

static void sensor_pos(double lat, double *px, double *py)
{
    const double fx = cos(g_th), fy = sin(g_th);
    *px = g_x + C.sensor_fwd * fx + lat * (-fy);
    *py = g_y + C.sensor_fwd * fy + lat * fx;
}

static double ramp(double v, double target, double dt)
{
    const double dv = C.accel * dt;
    if (target > v + dv) { return v + dv; }
    if (target < v - dv) { return v - dv; }
    return target;
}

static void physics_step(double dt)
{
    double px, py;

    if (g_goal_active) {
        g_cl = g_cr = g_goal_speed;
        if (g_goal_trav >= g_goal_mm) {
            g_goal_active = 0; g_goal_done = 1; g_cl = g_cr = 0.0;
        }
    }
    if (g_noise > 0.0) {   /* slow random wheel gain drift */
        g_gl += (frand() - 0.5) * 0.02; g_gr += (frand() - 0.5) * 0.02;
        g_gl += (1.0 - g_gl) * 0.002;   g_gr += (1.0 - g_gr) * 0.002;
        if (g_gl < 1 - g_noise) g_gl = 1 - g_noise; if (g_gl > 1 + g_noise) g_gl = 1 + g_noise;
        if (g_gr < 1 - g_noise) g_gr = 1 - g_noise; if (g_gr > 1 + g_noise) g_gr = 1 + g_noise;
    }
    g_vl = ramp(g_vl, g_cl * g_gl, dt);
    g_vr = ramp(g_vr, g_cr * g_gr, dt);
    {
        const double v = 0.5 * (g_vl + g_vr), w = C.turn_gain * (g_vr - g_vl) / g_track;
        g_x += v * cos(g_th) * dt;
        g_y += v * sin(g_th) * dt;
        g_th += w * dt;
        g_el += fabs(g_vl * dt) * g_tpm * C.enc_scale;
        g_er += fabs(g_vr * dt) * g_tpm * C.enc_scale;
        if (g_goal_active) { g_goal_trav += fabs(v * dt) * C.enc_scale; }
    }
    sensor_pos(C.left_lat, &px, &py);   g_pin[PIN_IR_LEFT] = is_black(px, py);
    sensor_pos(0.0, &px, &py);          g_pin[PIN_IR_CENTER] = is_black(px, py);
    sensor_pos(-C.right_lat, &px, &py);
    g_pin[PIN_IR_RIGHT] = is_black(px, py);
    if (py > -441.0 && py < -300.0) {   /* GP16 over the barcode's length */
        if (px < g_lat_min) g_lat_min = px;
        if (px > g_lat_max) g_lat_max = px;
    }
}

static void sim_advance_ms(int ms)
{
    /* step count, not "now < end": the 32-bit us clock wraps every 71.6 min */
    for (int step = 0; step < ms * 4; step++) {
        LineBarcodeData_t d;
        g_now_us += 250U;
        *g_timerawl = g_now_us;
        physics_step(250e-6);
        /* the barcode task (core 1) */
        process_sensors_distance_packed(0U, &d);
        if (d.scanned_code != S3_CODE_NONE) {
            const char ch = s3_code_to_char(d.scanned_code);
            g_decoded = ch;
            tk_snd_mbf(mbf_barcode, &ch, 1, TMO_POL);
        }
        if (strstr(g_lastlog, "junction,")) { g_junction_seen = 1; }
        if (strstr(g_lastlog, "line lost")) { g_lost = 1; }
        g_lastlog[0] = '\0';
        /* end: 250 mm out along an arm after the junction, or stuck */
        if (g_decode_only && g_y > -150.0) { longjmp(g_end, 3); }
        if (g_junction_seen && hypot(g_x, g_y) > 250.0) { longjmp(g_end, 1); }
        if ((int32_t)(g_now_us - g_t_end) > 0 || g_lost) { longjmp(g_end, 2); }
    }
}

/* ---------- stubs for the other subsystems ---------- */
BOOL s1_mission_go(void) { return TRUE; }
BOOL s2_ready(void) { return TRUE; }
BOOL s4_ready(void) { return TRUE; }
void subsystem5_get_status(subsystem5_status_t *r) { memset(r, 0, sizeof *r); }
void b4_get_status(b4_status_t *o) { memset(o, 0, sizeof *o); o->calibration_ready = 1; }
void b2_get_status(b2_status_t *o)
{
    memset(o, 0, sizeof *o);
    o->left_encoder_edges = (uint32_t)g_el;
    o->right_encoder_edges = (uint32_t)g_er;
}
bool b2_submit_command(const b2_command_t *c)
{
    if (c->type == B2_CMD_SPEED && !g_goal_active) { g_cl = c->left_mm_s; g_cr = c->right_mm_s; return true; }
    return false;
}
void b2_stop(void) { g_goal_active = 0; g_cl = g_cr = 0.0; }
bool b2_goal_start(b2_goal_t *g, const b2_command_t *c)
{
    (void)g;
    if (c->type != B2_CMD_MOVE_FORWARD) { return false; }
    g_goal_active = 1; g_goal_done = 0; g_goal_trav = 0.0;
    g_goal_mm = c->distance_mm; g_goal_speed = c->goal_speed_mm_s;
    return true;
}
b2_goal_result_t b2_goal_check(b2_goal_t *g)
{
    (void)g;
    if (g_goal_done && fabs(g_vl) < 1.0 && fabs(g_vr) < 1.0) { g_goal_done = 0; return B2_GOAL_DONE; }
    return B2_GOAL_RUNNING;
}

/* ---------- scenarios ---------- */
static const char *arm_name(double x, double y)
{
    if (hypot(x, y) < 200.0) { return "none (stuck near junction)"; }
    if (fabs(x) > fabs(y)) { return (x < 0) ? "LEFT" : "RIGHT"; }
    return (y > 0) ? "FORWARD" : "BACK (U-turn)";
}

static const char *run(char letter, const char *expect, int curve)
{
    static char res[96];
    int why;

    memset(g_pin, 0, sizeof g_pin);
    g_qn = 0; g_junction_seen = 0; g_lost = 0; g_decoded = '\0';
    g_x = C.x0; g_y = -APPROACH + 100.0; g_th = M_PI / 2;
    g_vl = g_vr = g_cl = g_cr = 0.0; g_goal_active = g_goal_done = 0;
    place_barcode(letter, -300.0 - 141.0);
    g_nextra = 0;
    if (curve) {
        /* a left-branching stub between the barcode and the junction: a
           spot where GP2 and GP4 are both on black, like a sharp left curve */
        g_extra_tape[0][0] = -60.0; g_extra_tape[0][1] = -159.0;
        g_extra_tape[0][2] = 0.0;   g_extra_tape[0][3] = -141.0;
        g_nextra = 1;
    }
    line_barcode_init();
    g_now_us += 1000000U;
    g_t_end = g_now_us + 60000000U;
    why = setjmp(g_end);
    if (why == 0) { line_follow_task(0, NULL); }
    snprintf(res, sizeof res, "%s", g_lost ? "LINE LOST" : arm_name(g_x, g_y));
    if (!g_decode_only) printf("  barcode '%c' decoded '%c'  expect %-13s got %-13s %s\n",
           letter, g_decoded ? g_decoded : '-', expect, res,
           (strcmp(res, expect) == 0) ? "PASS" : "FAIL");
    return (strcmp(res, expect) == 0) ? NULL : res;
}

int main(int argc, char **argv)
{
    static const struct { char l; const char *e; } cases[] = {
        { 'L', "LEFT" }, { 'R', "RIGHT" }, { 'U', "BACK (U-turn)" },
        { 'F', "FORWARD" }, { 'Q', "FORWARD" },   /* Q: not in the map */
    };
    static const car_cfg_t cfgs[] = {
        /* fwd  left right enc  turn  accel  x0 */
        { 60.0, 20.0, 45.0, 1.0, 1.0, 1000.0, 0.0 },   /* as S3 assumes */
        { 40.0, 20.0, 45.0, 1.0, 1.0, 1000.0, 0.0 },   /* sensors-to-axle  */
        { 80.0, 20.0, 45.0, 1.0, 1.0, 1000.0, 0.0 },   /*   not measured   */
        { 60.0, 30.0, 45.0, 1.0, 1.0, 1000.0, 0.0 },   /* wider GP2 gap */
        { 60.0, 20.0, 45.0, 1.0, 1.7, 1000.0, 0.0 },   /* 19 Nov turn error */
        { 60.0, 20.0, 45.0, 0.8, 1.0, 1000.0, 0.0 },   /* circumference -20%: */
        { 60.0, 20.0, 45.0, 1.2, 1.0, 1000.0, 0.0 },   /*  advance 75 / 50 mm */
        { 60.0, 20.0, 45.0, 1.0, 1.0,  400.0, 4.0 },   /* slow motors, offset */
    };
    int fails = 0, total = 0;

    g_verbose = (argc > 1 && strcmp(argv[1], "-v") == 0);
    map_timer();
    /* -w: start 100 s before the 32-bit us clock wraps (car on > 71.6 min) */
    g_now_us = (argc > 1 && strcmp(argv[argc - 1], "-w") == 0) ? (0xFFFFFFFFU - 100000000U) : 1000000U;
    *g_timerawl = g_now_us;
    line_barcode_create();
    subsystem3_init();
    set_mqtt_action_mapping('F', 'U', 'L', 'R');   /* dashboard "map FULR" */

    for (unsigned c = 0; c < sizeof cfgs / sizeof cfgs[0]; c++) {
        C = cfgs[c];
        printf("car: sensors %.0f mm ahead of axle, GP2 %.0f mm left, encoder x%.1f, turn x%.1f, accel %.0f, start offset %.0f mm\n",
               C.sensor_fwd, C.left_lat, C.enc_scale, C.turn_gain, C.accel, C.x0);
        for (unsigned k = 0; k < sizeof cases / sizeof cases[0]; k++) {
            total++;
            if (run(cases[k].l, cases[k].e, 0)) { fails++; }
        }
    }
    C = cfgs[0];
    printf("course with a left stub (GP2+GP4 both black) between barcode and junction:\n");
    total++;
    if (run('R', "RIGHT", 1)) { fails++; }

    printf("%d/%d junction runs correct\n", total - fails, total);

    /* Decode stress: the car drives past the barcode while following the
       line, one condition changed at a time from the base car. */
    {
        static const struct { const char *what; double spot, noise, right, x0; } d[] = {
            { "base car",                         0.0, 0.00, 45.0, 0.0 },
            { "bars 1 mm thinner (edge -0.5)",   -0.5, 0.00, 45.0, 0.0 },
            { "bars 1 mm wider (edge +0.5)",      0.5, 0.00, 45.0, 0.0 },
            { "bars 2 mm wider (edge +1.0)",      1.0, 0.00, 45.0, 0.0 },
            { "bars 2.8 mm wider (edge +1.4)",    1.4, 0.00, 45.0, 0.0 },
            { "wheel speed noise 10%",            0.0, 0.10, 45.0, 0.0 },
            { "wheel speed noise 20%",            0.0, 0.20, 45.0, 0.0 },
            { "start 6 mm off line (weaving)",    0.0, 0.00, 45.0, 6.0 },
            { "GP16 35 mm right",                 0.0, 0.00, 35.0, 0.0 },
            { "GP16 40 mm right",                 0.0, 0.00, 40.0, 0.0 },
            { "GP16 55 mm right",                 0.0, 0.00, 55.0, 0.0 },
            { "GP16 65 mm right",                 0.0, 0.00, 65.0, 0.0 },
            { "GP16 70 mm right",                 0.0, 0.00, 70.0, 0.0 },
            { "wider 1 mm + noise 10% + offset 4",0.5, 0.10, 45.0, 4.0 },
        };
        int wrong_total = 0;
        g_decode_only = 1;
        printf("barcode decode while line following (20 random letters each):\n");
        for (unsigned i = 0; i < sizeof d / sizeof d[0]; i++) {
            int ok = 0, wrong = 0, none = 0;
            double lo = 1e9, hi = -1e9;
            C = cfgs[0];
            C.right_lat = d[i].right; C.x0 = d[i].x0;
            g_spot = d[i].spot; g_noise = d[i].noise;
            for (int t = 0; t < 20; t++) {
                const char letter = (char)('A' + (int)(frand() * 25.99));
                g_gl = g_gr = 1.0;
                g_lat_min = 1e9; g_lat_max = -1e9;
                run(letter, "", 0);
                if (g_lat_min < lo) lo = g_lat_min;
                if (g_lat_max > hi) hi = g_lat_max;
                if (g_verbose) printf("    run letter %c decoded %c\n", letter, g_decoded ? g_decoded : '-');
                if (g_decoded == letter) ok++;
                else if (g_decoded) wrong++;
                else none++;
            }
            wrong_total += wrong;
            printf("  %-34s %2d/20 correct, %d wrong, %2d not decoded   GP16 went %.0f..%.0f mm right (strip 29..79)\n",
                   d[i].what, ok, wrong, none, lo, hi);
        }
        g_decode_only = 0; g_spot = 0.0; g_noise = 0.0;
        if (wrong_total) { fails++; }
    }
    return fails ? 1 : 0;
}
