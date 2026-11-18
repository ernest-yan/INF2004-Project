/**
 * CR1 S3 equivalence driver. run_host_tests.sh builds this TWICE: once
 * against the pre-CR1 source tree (git archive pre-cr1) and once against
 * the CR1 tree, then diffs the two outputs. line_barcode.c is compiled for
 * real; only the kernel, GPIO and the other subsystems are stubbed.
 *
 * It simulates the GP16 barcode sensor for thousands of random Code 39
 * scans ('*' letter '*', random letter, bar speed, narrow:wide ratio,
 * per-element jitter, both scan directions, some damaged scans) and prints
 * the letter each build decodes, plus the barcode -> movement mapping.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

/* ---------- stubs: app.h / app_hw.h / test_report.h / kernel ---------- */
#define APP_H
#define APP_HW_H
#define TEST_REPORT_H
#define APP_TEST            0
#define APP_BARCODE_ONLY    0
#define APP_MISSION_ENABLE  0
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
static int g_gpio16;
static inline uint32_t app_time_us_32(void) { return g_now_us; }
static inline ER tk_cre_mtx(const T_CMTX *c) { (void)c; return 1; }
static inline ER tk_loc_mtx(ID i, TMO t) { (void)i; (void)t; return E_OK; }
static inline ER tk_unl_mtx(ID i) { (void)i; return E_OK; }
static inline ER tk_cre_mbf(const T_CMBF *c) { (void)c; return 2; }
static inline ER tk_snd_mbf(ID i, const void *m, INT s, TMO t) { (void)i; (void)m; (void)s; (void)t; return E_OK; }
static inline ER tk_dly_tsk(INT t) { (void)t; return E_OK; }
static inline int tm_printf(const UB *f, ...) { (void)f; return 0; }
static inline int gpio_set_pin(unsigned int p, int m) { (void)p; (void)m; return 0; }
static inline int gpio_get_val(unsigned int p) { return (p == 16U) ? g_gpio16 : 1; }
static inline ID app_start_task(void (*f)(INT, void *), int pri, int prc, const char *n)
{ (void)f; (void)pri; (void)prc; (void)n; return 3; }

/* The code under test (its other-subsystem headers come from the same tree).
 * It reads the RP2040 TIMERAWL register at 0x40054028 directly, so that page
 * is mapped on the host and the fake clock is written into it. */
#include "line_barcode.c"

static volatile uint32_t *g_timerawl;
static void map_timer(void)
{
    void *p = mmap((void *)0x40054000UL, 4096U, PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    if (p != (void *)0x40054000UL)
    {
        printf("cannot map the fake RP2040 timer page\n");
        exit(2);
    }
    g_timerawl = (volatile uint32_t *)0x40054028UL;
}

/* ---------- driver ---------- */
static uint32_t g_rng = 4242U;
static uint32_t rnd(uint32_t n)
{
    g_rng = (g_rng * 1103515245U) + 12345U;
    return (g_rng >> 8) % n;
}

static char g_last_decoded;
static unsigned g_fail_events;      /* CR1 build: scan_fail_count changes */
static unsigned g_prev_fail;
static void step_until(uint32_t t_us)
{
    while (g_now_us < t_us)
    {
        LineBarcodeData_t d;
        g_now_us += 50U;                     /* barcode task polls fast */
        *g_timerawl = g_now_us;
        process_sensors_distance_packed(0U, &d);
#ifdef S3_CODE_NONE
        if (d.scanned_code != S3_CODE_NONE) { g_last_decoded = s3_code_to_char(d.scanned_code); }
        if (d.scan_fail_count != g_prev_fail)
        {
            g_fail_events += (d.scan_fail_count - g_prev_fail) & 3U;
            g_prev_fail = d.scan_fail_count;
        }
#else
        if (d.scanned_char != '\0') { g_last_decoded = d.scanned_char; }
#endif
    }
}

static uint16_t pattern_of(char c)
{
    for (unsigned i = 0U; i < sizeof(CODE39_TABLE) / sizeof(CODE39_TABLE[0]); i++)
    {
        if (CODE39_TABLE[i].character == c) { return CODE39_TABLE[i].pattern; }
    }
    return 0U;
}

int main(void)
{
    map_timer();
    line_barcode_create();
    line_barcode_init();
    g_now_us = 1000000U;
    g_gpio16 = 0;
    step_until(g_now_us + 500000U);

    for (int run = 0; run < 3000; run++)
    {
        const char letter = (char)('A' + rnd(26U));
        const uint32_t narrow = 2000U + rnd(20000U);         /* us */
        const uint32_t ratio10 = 20U + rnd(11U);             /* 2.0 .. 3.0 */
        const uint32_t jitter = rnd(25U);                    /* +- % */
        const int reverse = (int)rnd(2U);
        const int damage = (rnd(20U) == 0U);                 /* drop an element */
        uint16_t chars[3] = { pattern_of('*'), pattern_of(letter), pattern_of('*') };
        uint32_t el[29];
        int n = 0;
        for (int c = 0; c < 3; c++)
        {
            for (int b = 8; b >= 0; b--)
            {
                const uint32_t base = ((chars[c] >> b) & 1U) ? (narrow * ratio10) / 10U : narrow;
                const int32_t j = (int32_t)rnd(2U * jitter + 1U) - (int32_t)jitter;
                el[n++] = (uint32_t)((int32_t)base + ((int32_t)base * j) / 100);
            }
            if (c < 2) { el[n++] = narrow; }                   /* inter-char gap */
        }
        if (damage) { el[10] = el[10] / 3U; }
        /* bar = black = 1 first; elements alternate. */
        for (int k = 0; k < 29; k++)
        {
            const int idx = reverse ? (28 - k) : k;
            g_gpio16 = ((k % 2) == 0) ? 1 : 0;
            step_until(g_now_us + el[idx]);
        }
        g_gpio16 = 0;
        g_last_decoded = '\0';
        step_until(g_now_us + 800000U);   /* quiet timeout + cooldown */
        printf("scan %4d %c narrow=%5u ratio=%u.%u jit=%2u%% %s%s -> %c\n",
               run, letter, (unsigned)narrow, (unsigned)(ratio10 / 10U),
               (unsigned)(ratio10 % 10U), (unsigned)jitter,
               reverse ? "rev" : "fwd", damage ? " damaged" : "",
               (g_last_decoded != '\0') ? g_last_decoded : '-');
    }

    /* Barcode -> movement mapping for every letter and digit. */
    static const char maps[][4] = { {'C','D','A','B'}, {'F','U','L','R'},
                                    {'0','9','Z','A'}, {'\0','\0','\0','\0'} };
    for (unsigned m = 0U; m < 4U; m++)
    {
        set_mqtt_action_mapping(maps[m][0], maps[m][1], maps[m][2], maps[m][3]);
        printf("map %u:", m);
        for (char c = '0'; c <= 'Z'; c++)
        {
            if ((c > '9') && (c < 'A')) { continue; }
            printf("%d", (int)get_action_for_character(c));
        }
        printf(" nul=%d\n", (int)get_action_for_character('\0'));
    }
    fprintf(stderr, "failed-scan events: %u\n", g_fail_events);
    return 0;
}
