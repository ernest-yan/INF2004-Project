/**
 * CR1 host test for subsystem 1's team adapters (team_adapters.c, compiled
 * for real; the other subsystems' getters are stubs returning set values).
 *
 * Checks:
 *   - the system fault bitmask: every S2/S3/S4/S5 fault and Wi-Fi loss sets
 *     exactly its own SYS_FLT_* bit
 *   - the MQTT fault byte (TLM_FAULT_Sn) derived from it follows the same
 *     rules as before CR1 (line loss is not a fault)
 *   - values going into the packed team structs saturate at the field width
 *     instead of wrapping (speed, distance, up-speed, distances in cm)
 *   - barcode letter -> movement through S3's packed map
 *   - struct sizes
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* ---------- stubs: app.h / app_hw.h / kernel ---------- */
#define APP_H
#define APP_HW_H
#define TM_MQTT 1
#define IMPORT  extern
#define EXPORT
#define LOCAL   static
typedef int ID;
typedef int ER;
typedef int INT;
typedef int W;
typedef unsigned int UINT;
typedef unsigned int UW;
typedef unsigned short UH;
typedef unsigned char UB;
typedef int BOOL;
#define TRUE  (1)
#define FALSE (0)
#define E_OK  (0)
static inline int tm_printf(const UB *f, ...) { (void)f; return 0; }
static inline uint32_t app_time_us_32(void) { return 0U; }
static inline uint64_t app_time_us_64(void) { return 0U; }

#include "../../../app_program/subsystem1/team_adapters.c"
#include "../../../app_program/subsystem1/s1_internal.h"

/* ---------- the other subsystems, as stubs ---------- */
static b2_status_t g_m;
static b4_status_t g_i;
static subsystem5_status_t g_u;
static LineBarcodeData_t g_l;
static BOOL g_online = TRUE;
static uint32_t g_map;
void b2_get_status(b2_status_t *o) { *o = g_m; }
void b4_get_status(b4_status_t *o) { *o = g_i; }
void subsystem5_get_status(subsystem5_status_t *o) { *o = g_u; }
LineBarcodeData_t get_line_barcode_data_safe(void) { return g_l; }
BOOL s1_is_online(void) { return g_online; }
BOOL s2_ready(void) { return TRUE; }
void b2_stop(void) {}
void b2_emergency_brake(void) {}
bool b2_authorized_reset_while_stopped(void) { return true; }
bool b4_authorized_reset_while_stopped(void) { return true; }
void subsystem5_reset_fault_while_stopped(void) {}
void set_mqtt_action_mapping(char f, char u, char l, char r)
{
    g_map = (uint32_t)s3_char_to_code(f) | ((uint32_t)s3_char_to_code(u) << 6) |
            ((uint32_t)s3_char_to_code(l) << 12) | ((uint32_t)s3_char_to_code(r) << 18);
}
MovementAction_t get_action_for_character(char c)
{
    const uint32_t code = s3_char_to_code(c);
    if (code == 0U) { return MOVE_NONE; }
    if ((g_map & 0x3FU) == code) { return MOVE_FORWARD; }
    if (((g_map >> 6) & 0x3FU) == code) { return MOVE_UTURN; }
    if (((g_map >> 12) & 0x3FU) == code) { return MOVE_LEFT; }
    if (((g_map >> 18) & 0x3FU) == code) { return MOVE_RIGHT; }
    return MOVE_NONE;
}

static int g_fail, g_checks;
#define CHECK(c) do { ++g_checks; if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); ++g_fail; } } while (0)

static void clear_all(void)
{
    static const b2_status_t zm; static const b4_status_t zi;
    static const subsystem5_status_t zu; static const LineBarcodeData_t zl;
    g_m = zm; g_i = zi; g_u = zu; g_l = zl;
    g_l.line_detected = 1U;
    g_online = TRUE;
}

static uint8_t wire_faults(void)
{
    vehicle_status_t v;
    vehicle_get_status(&v);
    return v.fault_mask;
}

int main(void)
{
    /* sizes of the packed team structs */
    CHECK(sizeof(vehicle_status_t) == 2U);
    CHECK(sizeof(line_status_t) == 3U);     /* + letter, seen-not-decoded */
    CHECK(sizeof(motion_status_t) == 8U);
    CHECK(sizeof(imu_status_t) == 10U);
    CHECK(sizeof(ultra_status_t) == 8U);
    CHECK(sizeof(S1_STATS) == 8U);

    /* no fault */
    clear_all();
    CHECK(system_fault_mask() == 0U);
    CHECK(wire_faults() == 0U);

    /* every S2 fault -> its own bit, and TLM_FAULT_S2 */
    for (uint32_t f = 1U; f <= 4U; f++)
    {
        clear_all();
        g_m.fault_mask = B2_FAULT_BIT(f);
        CHECK(system_fault_mask() == (1U << (f - 1U)));
        CHECK(wire_faults() == TLM_FAULT_S2);
    }
    CHECK((SYS_FLT_MOTOR_STALL == (1U << (B2_FAULT_ENCODER_STALL - 1U))));

    /* S3 line loss: own bit, but NOT a wire fault (as before CR1) */
    clear_all();
    g_l.fault_mask = FAULT_LINE_LOSS;
    CHECK(system_fault_mask() == SYS_FLT_LINE_LOSS);
    CHECK(wire_faults() == 0U);

    /* every S4 fault */
    for (uint32_t f = 1U; f <= 3U; f++)
    {
        clear_all();
        g_i.fault_mask = B4_FAULT_BIT(f);
        CHECK(system_fault_mask() == (1U << (4U + f)));
        CHECK(wire_faults() == TLM_FAULT_S4);
    }
    CHECK(SYS_FLT_IMU_I2C == (1U << (5U + B4_FAULT_I2C - 1U)));

    /* every S5 fault, incl. no echo */
    for (uint32_t f = 1U; f <= 6U; f++)
    {
        clear_all();
        g_u.fault_mask = subsystem5_FAULT_BIT(f);
        CHECK(system_fault_mask() == (1U << (7U + f)));
        CHECK(wire_faults() == TLM_FAULT_S5);
        ultra_status_t us;
        ultra_get_status(&us);
        CHECK(us.fault == f);            /* MQTT 3-bit enum, unchanged */
    }
    CHECK(SYS_FLT_NO_ECHO == (1U << (8U + subsystem5_FAULT_MONITOR_SENSOR - 1U)));

    /* Wi-Fi loss */
    clear_all();
    g_online = FALSE;
    CHECK(system_fault_mask() == SYS_FLT_WIFI_LOSS);
    CHECK(wire_faults() == 0U);

    /* several at once: all bits kept */
    clear_all();
    g_m.fault_mask = B2_FAULT_BIT(B2_FAULT_ENCODER_STALL);
    g_i.fault_mask = B4_FAULT_BIT(B4_FAULT_I2C);
    g_u.fault_mask = subsystem5_FAULT_BIT(subsystem5_FAULT_MONITOR_SENSOR);
    g_online = FALSE;
    CHECK(system_fault_mask() == (SYS_FLT_MOTOR_STALL | SYS_FLT_IMU_I2C |
                                  SYS_FLT_NO_ECHO | SYS_FLT_WIFI_LOSS));
    CHECK(wire_faults() == (TLM_FAULT_S2 | TLM_FAULT_S4 | TLM_FAULT_S5));

    /* saturation into the packed fields */
    clear_all();
    {
        motion_status_t m;
        g_m.left_speed_ob = (uint32_t)(300 + B2_SPEED_OFFSET);
        g_m.right_speed_ob = (uint32_t)(-200 + B2_SPEED_OFFSET);  /* reverse */
        g_m.left_total_mm = 32767;
        g_m.right_total_mm = 32767;
        g_m.left_encoder_edges = 0x12345U;
        motion_get_status(&m);
        CHECK(m.speed_l_mm_s == 300U);
        CHECK(m.speed_r_mm_s == 200U);    /* direction not sent */
        CHECK(m.distance_cm == 3276U);
        CHECK(m.enc_l == 0x2345U);        /* 16-bit wrap, as on the wire */
    }
    {
        imu_status_t i;
        g_i.recent_up_speed_mm_s = 2000;  /* would wrap a 10-bit field */
        g_i.pitch_cdeg = -9000;
        g_i.max_hump_height_mm = 200U;
        g_i.lift_latched = 1U;
        g_i.lift_cause = B4_LIFT_OVERTURNED;
        imu_get_status(&i);
        CHECK(i.up_speed_mm_s == 1023U);
        CHECK(i.pitch_deg == -64);
        CHECK(i.hump_max_mm == 200U);
        CHECK(i.lift_cause == 2U);
        CHECK(i.lift_count == 1U);
    }
    {
        ultra_status_t u;
        g_u.front_valid = 0U;
        g_u.profile.left_clearance_valid = 1U;
        g_u.profile.left_clearance_mm = 4000U;
        ultra_get_status(&u);
        CHECK(u.nearest_cm == 0x1FFU);    /* no echo */
        CHECK(u.clear_l_cm == 400U);
    }

    /* barcode letter -> command through the packed map (example: map CDAB) */
    clear_all();
    barcode_set_mapping("CDAB");
    {
        line_status_t l;
        static const struct { char c; uint8_t cmd; } k[] = {
            {'A', BARCODE_LEFT}, {'B', BARCODE_RIGHT}, {'C', BARCODE_STRAIGHT},
            {'D', BARCODE_UTURN}, {'E', BARCODE_INVALID}, {'\0', BARCODE_NONE}
        };
        for (unsigned n = 0U; n < 6U; n++)
        {
            g_l.last_code = s3_char_to_code(k[n].c);
            line_get_status(&l);
            CHECK(l.last_barcode == k[n].cmd);
            CHECK(s3_code_to_char(l.letter_code) == k[n].c);   /* letter sent too */
        }
    }
    g_l.scan_fail_count = 3U;
    {
        line_status_t l;
        line_get_status(&l);
        CHECK(l.scan_fail_count == 3U);
    }
    for (char c = 'A'; c <= 'Z'; c++) { CHECK(s3_code_to_char(s3_char_to_code(c)) == c); }
    for (char c = '0'; c <= '9'; c++) { CHECK(s3_code_to_char(s3_char_to_code(c)) == c); }

    printf("S1 adapters + system fault mask host test: %d checks, %d failed\n",
           g_checks, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
