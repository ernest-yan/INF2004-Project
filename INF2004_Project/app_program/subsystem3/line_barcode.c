/*
 *	line_barcode.c
 *	Subsystem 3 -- line sensors + Code 39 barcode decoder.
 *
 *	This ONE file holds these things, in this order:
 *	  1. the driver / decoder
 *	  2. line following and barcode turns (line_follow_task)
 *	  3. the subsystem glue the template needs (subsystem3_init/_start,
 *	     the barcode task)
 *
 *	Ported from the Pico SDK version.  Every change is marked "PORT:".
 *
 *	Call rules (TEAM.md section 3: one owning task per peripheral):
 *	  - line_barcode_create()  once, from subsystem3_init()
 *	  - line_barcode_init() and process_sensors_distance_packed()
 *	    only from the barcode task
 *	  - get_line_barcode_data_safe() / set_mqtt_action_mapping()
 *	    may be called from any task
 */

#include "line_barcode.h"
#include "../test_report.h"
#include "../app_hw.h"
#include "../subsystem2/tkernel_adapter.h"
#include "../subsystem2/motion_cal.h"	/* B2_CAL_TURN_TICKS_PER_DEG_X10 */
#include "../subsystem4/tkernel_adapter.h"
#include "../subsystem5/ultrasonic.h"
#include "../subsystem1/subsystem1.h"	/* S1: s1_mission_go() (IF-18) */

/* =====================================================================
 *  1. Driver / decoder
 * ===================================================================== */

/* PORT: <stddef.h> is NOT included.  The kernel header tk/syslib.h already
 * defines size_t (as a signed int) and a second definition would not compile.
 * Plain int is used for the table loop below instead. */
#include <stdint.h>
#include <stdbool.h>

/*
 * PORT: replaces time_us_32().  The RP2040 TIMER peripheral counts
 * microseconds; TIMERAWL (offset 0x28) reads the low 32 bits without
 * latching, so it does not disturb the kernel's own use of the timer.
 */
#define RP2040_TIMERAWL		(*(volatile uint32_t *)0x40054028u)

static inline uint32_t time_us_32(void)
{
    return RP2040_TIMERAWL;
}

/*
 * PORT: replaces save_and_disable_interrupts()/restore_interrupts().
 * Disabling interrupts only stops the CURRENT core; with the dual-core
 * build another task may run on the other core, so a kernel mutex is used.
 */
static ID g_mtx = 0;

static void data_lock(void)
{
    if (g_mtx > 0) {
        tk_loc_mtx(g_mtx, TMO_FEVR);
    }
}

static void data_unlock(void)
{
    if (g_mtx > 0) {
        tk_unl_mtx(g_mtx);
    }
}

ER line_barcode_create(void)
{
    T_CMTX cmtx = {0};

    cmtx.mtxatr = TA_INHERIT;	/* priority inheritance */
    g_mtx = tk_cre_mtx(&cmtx);
    return (g_mtx < E_OK) ? g_mtx : E_OK;
}

/*
 * Code 39 patterns are represented as nine elements in scan order:
 * 1 = wide, 0 = narrow. The standard has 3 wide elements per character.
 *
 * These are the same A-Z patterns used by the original project.
 * '*' is the Code 39 start/stop character; it frames the letter (see
 * decode_collected_scan) and is not emitted.
 */
typedef struct {
    char character;
    uint16_t pattern;
} Code39Map;

static const Code39Map CODE39_TABLE[] = {
    {'*', 0b010010100},
    {'A', 0b100001001}, {'B', 0b001001001}, {'C', 0b101001000},
    {'D', 0b000011001}, {'E', 0b100011000}, {'F', 0b001011000},
    {'G', 0b000001101}, {'H', 0b100001100}, {'I', 0b001001100},
    {'J', 0b000011100}, {'K', 0b100000011}, {'L', 0b001000011},
    {'M', 0b101000010}, {'N', 0b000010011}, {'O', 0b100010010},
    {'P', 0b001010010}, {'Q', 0b000000111}, {'R', 0b100000110},
    {'S', 0b001000110}, {'T', 0b000010110}, {'U', 0b110000001},
    {'V', 0b011000001}, {'W', 0b111000000}, {'X', 0b010010001},
    {'Y', 0b110010000}, {'Z', 0b011010000}
};

#define CODE39_STAR_PATTERN	0b010010100
#define CODE39_FRAME_ELEMENTS	29	/* '*' gap letter gap '*' */

static LineBarcodeData_t g_system_data = {0};

/* CR1: the four MQTT map letters as 6-bit codes in ONE word (24 bits):
 * forward bits 0-5, U-turn 6-11, left 12-17, right 18-23.  Written with a
 * single aligned 32-bit store, so a reader on another task can never see
 * half an old and half a new mapping (the four separate chars could). */
#define S3_MAP_FWD      (0U)
#define S3_MAP_UTURN    (6U)
#define S3_MAP_LEFT     (12U)
#define S3_MAP_RIGHT    (18U)
#define S3_MAP_MASK     (0x3FU)
static volatile uint32_t g_mqtt_map = 0U;

/* One duration per completed HIGH/LOW element.
 * CR1: uint16_t in 8 us units (max 524 ms, saturating) instead of uint32_t
 * microseconds: half the RAM.  A 1 mm bar at 70 mm/s is ~14 000 us, so the
 * 8 us step is far below the decoder's narrow/wide margin. */
#define S3_WIDTH_UNIT_US	(8U)
static uint16_t g_widths[MAX_BARS];
static uint32_t g_last_edge_time = 0;
static uint32_t g_edge_pending_since = 0;
static uint32_t g_last_success_decode_time = 0;

/* CR1: barcode-task state.  Written ONLY by the barcode task (single
 * writer, so the bit-field read-modify-write cannot race).  scan_active is
 * also READ by line_follow_task on the other core: hence volatile. */
static volatile struct {
    uint8_t width_count : 6;		/* 0..MAX_BARS (60) */
    uint8_t last_sensor_state : 1;
    uint8_t sensor_state_initialized : 1;
    uint8_t edge_pending : 1;		/* GP16 changed, not yet held */
    uint8_t have_successful_decode : 1;
    uint8_t scan_active : 1;		/* GP16 is collecting a barcode (first
					   edge until the quiet timeout) */
    uint8_t : 5;
} g_bc;

/* Reverse exactly nine bits so scans in either direction can be matched. */
static uint16_t reverse_9bits(uint16_t value)
{
    uint16_t reversed = 0;
    for (int i = 0; i < 9; ++i) {
        if (value & (1u << i)) {
            reversed |= (uint16_t)(1u << (8 - i));
        }
    }
    return reversed;
}

void set_mqtt_action_mapping(char char_fwd,
                             char char_uturn,
                             char char_left,
                             char char_right)
{
    const uint32_t map =
        ((uint32_t)s3_char_to_code(char_fwd)   << S3_MAP_FWD) |
        ((uint32_t)s3_char_to_code(char_uturn) << S3_MAP_UTURN) |
        ((uint32_t)s3_char_to_code(char_left)  << S3_MAP_LEFT) |
        ((uint32_t)s3_char_to_code(char_right) << S3_MAP_RIGHT);
    data_lock();	/* PORT: was save_and_disable_interrupts() */
    g_mqtt_map = map;	/* CR1: one store */
    data_unlock();	/* PORT: was restore_interrupts() */
}

MovementAction_t get_action_for_character(char c)
{
    const uint32_t code = s3_char_to_code(c);
    const uint32_t map = g_mqtt_map;	/* CR1: one consistent read */

    if (code == S3_CODE_NONE) return MOVE_NONE;

    if (((map >> S3_MAP_FWD) & S3_MAP_MASK) == code)
        return MOVE_FORWARD;
    if (((map >> S3_MAP_UTURN) & S3_MAP_MASK) == code)
        return MOVE_UTURN;
    if (((map >> S3_MAP_LEFT) & S3_MAP_MASK) == code)
        return MOVE_LEFT;
    if (((map >> S3_MAP_RIGHT) & S3_MAP_MASK) == code)
        return MOVE_RIGHT;

    return MOVE_NONE;
}

void line_barcode_init(void)
{
    /* PORT: gpio_init() + gpio_set_dir(GPIO_IN) -> gpio_set_pin(GPIO_MODE_IN) */
    gpio_set_pin(PIN_IR_LEFT,   GPIO_MODE_IN);
    gpio_set_pin(PIN_IR_CENTER, GPIO_MODE_IN);
    gpio_set_pin(PIN_IR_RIGHT,  GPIO_MODE_IN);

    g_bc.width_count = 0;
    g_last_edge_time = 0;
    g_bc.sensor_state_initialized = 0U;
    g_bc.have_successful_decode = 0U;
}

LineBarcodeData_t get_line_barcode_data_safe(void)
{
    data_lock();	/* PORT */
    LineBarcodeData_t copy = g_system_data;
    data_unlock();	/* PORT */
    return copy;
}

/* Second smallest of the n (<= 5) values in[first], in[first + 2], ... */
static uint32_t second_smallest(const uint32_t in[9], int first, int n)
{
    uint32_t lo = UINT32_MAX;
    uint32_t lo2 = UINT32_MAX;

    for (int i = 0; i < n; ++i) {
        const uint32_t v = in[first + (2 * i)];
        if (v < lo) {
            lo2 = lo;
            lo = v;
        } else if (v < lo2) {
            lo2 = v;
        }
    }
    return lo2;
}

/*
 * Bar-widening correction, then sort nine durations, estimate a narrow/wide
 * boundary from the gap between the 6th and 7th sorted values, and classify
 * the corrected ordered durations.
 *
 * The IR spot makes GP16 see black early and leave it late, so bars read
 * wider and spaces narrower by the same amount (1-2 mm).  Sorted together,
 * a widened narrow bar then matches a shrunk wide space, and the '*' frame
 * is lost (simulated: 1.5 mm already failed most scans, 2026-11-19).  Every
 * letter and '*' has 3 narrow bars (of 5) and 3 narrow spaces (of 4), so the
 * middle narrow bar minus the middle narrow space is twice that offset: half
 * of it comes off each bar and goes onto each space.  On a clean scan the
 * offset is ~0 and nothing changes.
 *
 * A candidate is rejected if there is no separation between narrow and wide
 * durations, or if the result does not have exactly three wide elements.
 */
static bool classify_nine(const uint32_t raw[9], uint16_t *pattern_out)
{
    uint32_t in[9];
    uint32_t sorted[9];

    for (int i = 0; i < 9; ++i) {
        if (raw[i] == 0) return false;
    }

    const int32_t offset = ((int32_t)second_smallest(raw, 0, 5) -
                            (int32_t)second_smallest(raw, 1, 4)) / 2;

    for (int i = 0; i < 9; ++i) {
        const int32_t v = (int32_t)raw[i] + (((i % 2) == 0) ? -offset : offset);
        if (v <= 0) return false;
        in[i] = (uint32_t)v;
        sorted[i] = in[i];
    }

    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8 - i; ++j) {
            if (sorted[j] > sorted[j + 1]) {
                uint32_t t = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = t;
            }
        }
    }

    /* Six narrow and three wide elements are expected in Code 39. */
    uint32_t max_narrow = sorted[5];
    uint32_t min_wide = sorted[6];

    if (min_wide <= max_narrow) return false;

    /*
     * Reject very weak separation. This ratio is deliberately more tolerant
     * than a strict 2:1 threshold because real sensor timing is noisy.
     */
    if ((uint64_t)min_wide * 10u < (uint64_t)max_narrow * 13u)
        return false;

    uint32_t threshold = max_narrow + ((min_wide - max_narrow) / 2u);
    uint16_t pattern = 0;
    unsigned wide_count = 0;

    for (int i = 0; i < 9; ++i) {
        if (in[i] > threshold) {
            pattern |= (uint16_t)(1u << (8 - i));
            ++wide_count;
        }
    }

    if (wide_count != 3) return false;

    *pattern_out = pattern;
    return true;
}

/* Exact match only: the scan direction is already known from the frame. */
static char lookup_code39(uint16_t pattern)
{
    for (int i = 0; i < (int)(sizeof(CODE39_TABLE) / sizeof(CODE39_TABLE[0])); ++i) {
        if (CODE39_TABLE[i].pattern == pattern) {
            /* Start/stop '*' is framing, not an application character. */
            return CODE39_TABLE[i].character == '*' ? '\0'
                                                     : CODE39_TABLE[i].character;
        }
    }

    return '\0';
}

static bool classify_at(uint8_t start, uint16_t *pattern_out)
{
    uint32_t candidate[9];

    for (int i = 0; i < 9; ++i) {
        candidate[i] = g_widths[start + i];	/* 8 us units: ratios kept */
    }
    return classify_nine(candidate, pattern_out);
}

/*
 * A barcode is '*' gap letter gap '*'.  A letter is only accepted between two
 * start/stop characters, because nine durations taken at the wrong offset can
 * look like another letter.  The frame also gives the scan direction: read
 * backwards '*' comes out mirrored, and the letter is mirrored back before the
 * lookup.  Without this K/U, N/Y, O/X, Q/W, R/V and T/Z could not be told
 * apart, because each pair are mirror images of each other.
 */
static char decode_collected_scan(uint8_t *p_result)
{
    *p_result = (uint8_t)S3_SCAN_SHORT;
    if (g_bc.width_count < CODE39_FRAME_ELEMENTS) return '\0';

    *p_result = (uint8_t)S3_SCAN_NO_FRAME;
    for (uint8_t start = 0;
         (uint16_t)start + CODE39_FRAME_ELEMENTS <= g_bc.width_count; ++start) {
        uint16_t first, letter, last;
        bool reversed = false;

        if (!classify_at(start, &first) ||
            !classify_at((uint8_t)(start + 20u), &last)) {
            continue;
        }

        if (first != CODE39_STAR_PATTERN || last != CODE39_STAR_PATTERN) {
            if (reverse_9bits(first) != CODE39_STAR_PATTERN ||
                reverse_9bits(last) != CODE39_STAR_PATTERN) {
                continue;
            }
            reversed = true;
        }

        /* Framed by '*' at both ends: from here a failure is the letter. */
        *p_result = (uint8_t)S3_SCAN_BAD_LETTER;
        if (!classify_at((uint8_t)(start + 10u), &letter)) {
            continue;
        }
        if (reversed) {
            letter = reverse_9bits(letter);
        }

        char c = lookup_code39(letter);
        if (c != '\0') {
            *p_result = (uint8_t)S3_SCAN_OK;
            return c;
        }
    }

    return '\0';
}

static void append_duration(uint32_t duration_us)
{
    if (duration_us < MIN_EDGE_US) {
        return; /* Ignore very short noise pulses. */
    }

    /* CR1: round to 8 us units, saturating at 65535 (524 ms). */
    const uint32_t units = (duration_us + (S3_WIDTH_UNIT_US / 2U)) /
                           S3_WIDTH_UNIT_US;
    const uint16_t duration = (units > UINT16_MAX) ? (uint16_t)UINT16_MAX
                                                   : (uint16_t)units;

    if (g_bc.width_count < MAX_BARS) {
        g_widths[g_bc.width_count] = duration;
        g_bc.width_count++;
    } else {
        /* Keep the newest measurements in a rolling buffer. */
        for (uint8_t i = 1; i < MAX_BARS; ++i) {
            g_widths[i - 1] = g_widths[i];
        }
        g_widths[MAX_BARS - 1] = duration;
    }
}

void process_sensors_distance_packed(uint32_t current_ticks,
                                     LineBarcodeData_t *out_data)
{
    (void)current_ticks; /* Kept for API compatibility with the existing code. */

    /* PORT: gpio_get() -> gpio_get_val() */
    bool ir_left = gpio_get_val(PIN_IR_LEFT);
    bool ir_center = gpio_get_val(PIN_IR_CENTER);
    bool ir_right = gpio_get_val(PIN_IR_RIGHT);

    uint32_t now_us = time_us_32();
    char decoded_char = '\0';
    bool scan_failed = false;	/* a barcode-sized scan gave no letter */
    bool scan_done = false;	/* a scan was decoded this call (any result) */
    uint8_t scan_elems = 0u;	/* its bars + spaces */
    uint8_t scan_result = (uint8_t)S3_SCAN_OK;

    /*
     * Seed the edge timer from the first sample. Do not treat startup time as
     * the duration of the first barcode element.
     */
    if ((g_bc.sensor_state_initialized == 0U)) {
        g_bc.last_sensor_state = ir_right ? 1U : 0U;
        g_last_edge_time = now_us;
        g_bc.sensor_state_initialized = 1U;
    } else if ((ir_right ? 1U : 0U) != g_bc.last_sensor_state) {
        /*
         * Debounce: accept an edge only once the new level has held for
         * S3_EDGE_DEBOUNCE_US.  The edge is timestamped where the change
         * began, so real widths are not shortened; flickers at a bar edge
         * are dropped instead of becoming two fake elements.
         */
        if ((g_bc.edge_pending == 0U)) {
            g_bc.edge_pending = 1U;
            g_edge_pending_since = now_us;
        }
        if ((uint32_t)(now_us - g_edge_pending_since) >= S3_EDGE_DEBOUNCE_US) {
            append_duration(g_edge_pending_since - g_last_edge_time);
            g_last_edge_time = g_edge_pending_since;
            g_bc.last_sensor_state = ir_right ? 1U : 0U;
            g_bc.edge_pending = 0U;
        }
    } else {
        g_bc.edge_pending = 0U;	/* flicker ended: back to the old level */
    }

    uint32_t quiet_time = now_us - g_last_edge_time;

    /*
     * Finalize when no transition has occurred for TIMEOUT_US. Decode from the
     * collected edge durations and then clear the scan buffer.
     */
#if S3_DEBUG_SCAN
    {
        static uint32_t dbg_last_us = 0;
        /* Only while idle: printing mid-scan would distort bar widths. */
        if (g_bc.width_count == 0 && now_us - dbg_last_us >= 2000000u) {
            dbg_last_us = now_us;
            tm_printf((UB*)"[S3 dbg] idle  GP16=%d  L=%d  C=%d\n",
                      ir_right ? 1 : 0, ir_left ? 1 : 0, ir_center ? 1 : 0);
        }
        if (quiet_time >= TIMEOUT_US && g_bc.width_count > 0) {
            /* Element 0 is the idle time before the first edge. */
            tm_printf((UB*)"[S3 dbg] scan ended: %d elements (need %d+), us:",
                      g_bc.width_count, CODE39_FRAME_ELEMENTS);
            for (uint8_t i = 0; i < g_bc.width_count; ++i) {
                tm_printf((UB*)" %u", (unsigned)g_widths[i] * S3_WIDTH_UNIT_US);
            }
            tm_printf((UB*)"\n");
        }
    }
#endif
    if (g_bc.width_count >= S3_SCAN_REPORT_MIN && quiet_time >= TIMEOUT_US) {
        bool cooldown_over = (g_bc.have_successful_decode == 0U) ||
            (uint32_t)(now_us - g_last_success_decode_time) >=
                DECODE_COOLDOWN_US;

        if (cooldown_over) {
            decoded_char = decode_collected_scan(&scan_result);
            scan_done = true;
            /* Element 0 is the idle time before the first bar. */
            scan_elems = (uint8_t)(g_bc.width_count - 1u);
            if (decoded_char != '\0') {
                g_last_success_decode_time = now_us;
                g_bc.have_successful_decode = 1U;
            } else {
                scan_failed = true;	/* reported to S1 below */
            }
        }
#if APP_BARCODE_ONLY
        if (decoded_char == '\0') {
            tm_printf((UB*)"[barcode] no letter (%d bars+spaces seen, a full"
                      " barcode is %d)\n", g_bc.width_count - 1,
                      CODE39_FRAME_ELEMENTS);
        }
#endif

        g_bc.width_count = 0;
    } else if (quiet_time >= TIMEOUT_US && g_bc.width_count > 0) {
        /* Incomplete scan; discard it after the quiet timeout. */
        g_bc.width_count = 0;
    }
    g_bc.scan_active = (g_bc.width_count > 0U) ? 1U : 0U;

    /* Line tracking: retain the original active-HIGH logic. */
    uint8_t line_state = (ir_left || ir_center) ? 1u : 0u;
    uint8_t cross_state = (ir_left && ir_center) ? 1u : 0u;

    data_lock();	/* PORT: was save_and_disable_interrupts() */

    g_system_data.ir_left_raw = ir_left ? 1u : 0u;
    g_system_data.ir_center_raw = ir_center ? 1u : 0u;
    g_system_data.ir_right_raw = ir_right ? 1u : 0u;
    g_system_data.line_detected = line_state;
    g_system_data.cross_junction = cross_state;
    g_system_data.scanned_code = s3_char_to_code(decoded_char);
    if (decoded_char != '\0') {	/* S1: kept for slow readers (telemetry) */
        g_system_data.barcode_count++;
        g_system_data.last_code = s3_char_to_code(decoded_char);
    }

    g_system_data.fault_mask = line_state ? 0U : FAULT_LINE_LOSS;
    if (scan_done) {	/* set before either counter moves, same lock */
        g_system_data.last_scan_elems = scan_elems;
        g_system_data.last_scan_result = scan_result;
    }
    if (scan_failed) {
        g_system_data.scan_fail_count++;	/* 2-bit, wraps */
    }

    if (out_data != NULL) {
        *out_data = g_system_data;
    }

    /* One-cycle event: the caller receives the character through out_data. */
    g_system_data.scanned_code = S3_CODE_NONE;

    data_unlock();	/* PORT: was restore_interrupts() */
}


/* =====================================================================
 *  2. Line following and barcode turns
 *
 *  Runs in its own task on core 0, so the barcode task on core 1 keeps
 *  measuring bar widths undisturbed.  Sensor logic with only two line
 *  sensors (1 = on the line, same active-HIGH logic as above):
 *    centre          on the line: straight (left too = junction: straight)
 *    left only       the line is to the left: steer left
 *    neither         sweep: first toward the side the last correction turned
 *                    (curves keep bending that way), after S3_SEARCH_SWAP_MS
 *                    the other way, give up after S3_LINE_LOST_MS
 *
 *  A decoded barcode is only stored.  Its movement runs at the next
 *  cross-junction (both sensors on the line), after driving on until the
 *  wheels are over the junction.  Then the stored letter is cleared.
 *  The turn is a pivot that counts lines under the centre sensor (no
 *  encoder angle): left/right stop on the first line after leaving the
 *  current one, a U-turn passes the left branch and stops on the second.
 *
 *  S5 has priority over the wheels: nothing here moves them unless S5 is
 *  monitoring normally with no stop request.
 * ===================================================================== */

EXPORT ID	mbf_barcode;	/* IF-01, see line_barcode.h */

#if APP_MISSION_ENABLE

#if S3_BENCH_MAPPING
#warning "S3_BENCH_MAPPING=1: barcode mapping is hard-coded until MQTT sends one. Set it to 0 once subsystem 1 is in."
#endif

typedef enum {
	LF_IDLE = 0,	/* wheels stopped, waiting for a line */
	LF_FOLLOW,
	LF_ADVANCE,	/* driving the wheels onto the junction */
	LF_TURN		/* pivoting, counting lines, for the stored barcode */
} line_follow_state_t;

/* CR1: line_follow_task flags, one bit each (were BOOLs, 4 bytes each).
 * Written ONLY by line_follow_task (single writer: no RMW race).
 * following is also read by S5 from another task (s3_line_following()):
 * hence volatile. */
static volatile struct {
	uint8_t following : 1;
	uint8_t hold : 1;		/* holding straight for a barcode scan */
	uint8_t searching : 1;
	uint8_t turning : 1;
	uint8_t last_go_left : 1;
	uint8_t exiting : 1;
	uint8_t : 2;
} s_lf;

EXPORT BOOL s3_line_following(void)
{
	return (s_lf.following != 0U) ? TRUE : FALSE;
}

/* TRUE when S3 may drive: the operator sent START (S1), no S5 obstacle
   handling, no latched S2 brake, and S4 calibrated (a RESET makes S4
   recalibrate, and moving then would ruin it). */
static BOOL line_follow_allowed(void)
{
	subsystem5_status_t s5;
	b2_status_t s2;
	b4_status_t s4;

	subsystem5_get_status(&s5);
	b2_get_status(&s2);
	b4_get_status(&s4);
	return s1_mission_go() &&
	       (s5.state == subsystem5_STATE_MONITOR) && (s5.stop_required == 0U) &&
	       (s2.brake_latched == 0U) && (s2.state != B2_FAULT) &&
	       (s4.calibration_ready != 0U);
}

static void line_follow_drive(const LineBarcodeData_t *d, BOOL *line_is_left)
{
	static uint32_t hold_since = 0u;
	static uint32_t search_since = 0u;
	static uint32_t last_call = 0u;
	static uint32_t steer_since = 0u;
	static uint32_t exit_since = 0u;
	b2_command_t cmd = {0};

	/* Not called for a while (stopped, junction turn): start fresh. */
	if ((uint32_t)(app_time_us_32() - last_call) >
	    (10u * S3_FOLLOW_PERIOD_MS * 1000u)) {
		s_lf.searching = 0U;
		s_lf.turning = 0U;
		s_lf.exiting = 0U;
	}
	last_call = app_time_us_32();

	/* Hold straight while a barcode is being scanned: the barcode sensor is
	   on the right, so steering would change its speed mid-scan and stretch
	   some bars (inner wheel ~0 mm/s), and the letter would not decode.
	   Capped so stray GP16 edges cannot lock the steering.
	   Only while the centre sensor is still on the line: on a right-hand
	   curve the line itself passes under GP16, and holding straight then
	   drives the car off the curve. */
	if ((g_bc.scan_active != 0U) && d->ir_center_raw) {
		if (!s_lf.hold) {
			s_lf.hold = 1U;
			hold_since = app_time_us_32();
		}
	} else {
		s_lf.hold = 0U;
	}

	cmd.type = B2_CMD_SPEED;
	if (s_lf.hold && ((uint32_t)(app_time_us_32() - hold_since) <
		     (S3_SCAN_HOLD_MAX_MS * 1000u))) {
		cmd.left_mm_s = S3_LINE_SPEED_MM_S;
		cmd.right_mm_s = S3_LINE_SPEED_MM_S;
	} else if (d->ir_center_raw) {
		/* If the line now leaves the centre and the left sensor does not
		   see it, it went right (or the car crossed it while turning
		   left), so the next search starts to the right.  The older
		   "search the last turn side first" memory was tuned with GP2/GP4
		   crossed; with correct wiring it searched the wrong way after
		   every overshoot (2026-11-16 19:44 log: left/right sweeps for
		   10 s with the centre never settling on the line). */
		*line_is_left = FALSE;
		if (s_lf.turning) {
			s_lf.turning = 0U;
			s_lf.exiting = 1U;
			exit_since = app_time_us_32();
		}
		/* Soft exit: on a curve the line keeps bending, so snapping to
		   straight the moment the centre finds it makes the car leave the
		   line again on the same side (straight/steer/straight zig-zag).
		   Keep a gentle arc the same way for S3_STEER_EXIT_MS, unless the
		   sensor on the other side sees the line (overshot: go straight). */
		if (s_lf.exiting &&
		    ((uint32_t)(app_time_us_32() - exit_since) <
		     (S3_STEER_EXIT_MS * 1000u)) &&
		    !(s_lf.last_go_left ? d->ir_right_raw : d->ir_left_raw)) {
			cmd.left_mm_s = s_lf.last_go_left ? S3_STEER_EXIT_INNER_MM_S
						     : S3_LINE_SPEED_MM_S;
			cmd.right_mm_s = s_lf.last_go_left ? S3_LINE_SPEED_MM_S
						      : S3_STEER_EXIT_INNER_MM_S;
		} else {
			s_lf.exiting = 0U;
			cmd.left_mm_s = S3_LINE_SPEED_MM_S;
			cmd.right_mm_s = S3_LINE_SPEED_MM_S;
		}
	} else {
		s_lf.exiting = 0U;
		BOOL go_left = *line_is_left;

		if (d->ir_left_raw) {
			*line_is_left = TRUE;
			go_left = TRUE;
			s_lf.searching = 0U;
		} else if (d->ir_right_raw) {
			/* Centre lost and GP16 (right of the centre) sees black:
			   the line has bent right under the barcode sensor. */
			*line_is_left = FALSE;
			go_left = FALSE;
			s_lf.searching = 0U;
		} else {
			/* Neither sensor: the side is only a guess (the line can sit
			   in the gap between the left and centre sensors), so sweep:
			   the guessed side first, then back past the start the other
			   way until S3_LINE_LOST_MS. */
			if (!s_lf.searching) {
				s_lf.searching = 1U;
				search_since = app_time_us_32();
			}
			if ((uint32_t)(app_time_us_32() - search_since) >=
			    (S3_SEARCH_SWAP_MS * 1000u)) {
				go_left = !*line_is_left;
			}
		}
		/* Gentle arc first; if the centre has not found the line again
		   soon, slow the inner wheel to -S3_STEER_HARD_MM_S (0 = stopped:
		   turn about that wheel) for curves tighter than the arc
		   (~16 cm radius) and for s_lf.searching. */
		{
			int16_t inner = S3_STEER_INNER_MM_S;

			if (!s_lf.turning || (go_left != s_lf.last_go_left)) {
				steer_since = app_time_us_32();
			} else if ((uint32_t)(app_time_us_32() - steer_since) >=
				   (S3_STEER_HARD_MS * 1000u)) {
				inner = -S3_STEER_HARD_MM_S;
			}
			cmd.left_mm_s = go_left ? inner : S3_STEER_SPEED_MM_S;
			cmd.right_mm_s = go_left ? S3_STEER_SPEED_MM_S : inner;
		}
		s_lf.turning = 1U;
		s_lf.last_go_left = go_left ? 1U : 0U;
	}
	if (d->ir_center_raw) {
		s_lf.searching = 0U;
	}
	(void)b2_submit_command(&cmd);
}

/* Goals are only accepted from READY, so stop the speed tracking first. */
static BOOL line_follow_goal(b2_goal_t *goal, b2_cmd_type_t type,
			     uint16_t amount)
{
	b2_command_t cmd = {0};

	cmd.type = type;
	if (type == B2_CMD_MOVE_FORWARD) {
		cmd.distance_mm = amount;
	} else {
		cmd.turn_degrees = amount;
	}
	cmd.goal_speed_mm_s = S3_TURN_SPEED_MM_S;
	cmd.timeout_ms = S3_TURN_TIMEOUT_MS;

	b2_stop();
	return b2_goal_start(goal, &cmd);
}

static void line_follow_task(INT stacd, void *exinf)
{
	line_follow_state_t state = LF_IDLE;
	LineBarcodeData_t d;
	b2_goal_t goal;
	BOOL line_is_left = FALSE;
	BOOL junction_armed = TRUE;
	uint32_t lost_since = 0u;
	BOOL after_s5 = FALSE;	/* held by an S5 obstacle bypass */
	uint32_t junction_since = 0u;
	uint32_t turn_since = 0u;
	BOOL turn_left_start = FALSE;	/* centre has left the starting line */
	BOOL turn_center = FALSE;	/* debounced centre sensor */
	uint8_t turn_debounce = 0u;
	uint8_t turn_skip = 0u;		/* lines still to pass before stopping */
	uint32_t turn_edges0 = 0u;	/* S2 encoder edges (L+R) at turn start */
	BOOL on_junction = FALSE;
	MovementAction_t action = MOVE_NONE;
	char stored = '\0';
	char ch;

	(void)stacd; (void)exinf;

	/* Moving during S4's calibration would ruin it, and without S4 there is
	   no impact brake, so wait for both before the first move. */
	(void)tk_dly_tsk(S3_START_DELAY_MS);
	tm_printf((UB*)"[S3] waiting for S2 ready and S4 calibration\n");
	for (;;) {
		b4_status_t s4;

		if (s2_ready() && s4_ready()) {
			b4_get_status(&s4);
			if (s4.calibration_ready != 0U) {
				break;
			}
		}
		(void)tk_dly_tsk(100);
	}
	tm_printf((UB*)"[S3] line following armed\n");

	while (1) {
		d = get_line_barcode_data_safe();

		if (!line_follow_allowed()) {
			subsystem5_status_t s5;

			subsystem5_get_status(&s5);
			/* 20 Nov: remember an S5 bypass (it ends by stopping the car
			   on the line); a FAULT or a STOP from S1 does not count. */
			if (!s1_mission_go() || (s5.state == subsystem5_STATE_FAULT)) {
				after_s5 = FALSE;
			} else if (s5.state != subsystem5_STATE_MONITOR) {
				after_s5 = TRUE;
			} else {
				/* unchanged */
			}
			if (state != LF_IDLE) {
				b2_stop();
				state = LF_IDLE;
				tm_printf((UB*)"[S3] holding: no START (S1), S4 calibrating, or S5 or S2 needs the wheels stopped\n");
			}
		} else if (state == LF_IDLE) {
			/* After an S5 bypass, follow at once: the bypass stops the car
			   on the line, and if it rolled just off the tape, the 2.5 s
			   line search (S3_LINE_LOST_MS) finds it again. Before, S3
			   waited for a sensor on the line and the car stood still. */
			if (d.line_detected || after_s5) {
				state = LF_FOLLOW;
				lost_since = app_time_us_32();
				tm_printf((UB*)(after_s5 ? "[S3] back from the bypass, following\n"
						       : "[S3] line found, following\n"));
				after_s5 = FALSE;
			}
		}

		/* Store the newest barcode for the next junction. */
		while (tk_rcv_mbf(mbf_barcode, &ch, TMO_POL) == 1) {
			stored = ch;
			tm_printf((UB*)"[S3] barcode '%c' stored\n", ch);
		}

		if (state == LF_ADVANCE) {
			const b2_goal_result_t result = b2_goal_check(&goal);

			if (result == B2_GOAL_DONE) {
				/* Wheels on the junction: pivot and count lines. */
				state = LF_TURN;
				turn_since = app_time_us_32();
				turn_left_start = FALSE;
				turn_center = TRUE;	/* assume on the line we came on */
				turn_debounce = 0u;
				/* A junction is only detected with the left sensor on
				   black, so a left branch always exists; a U-turn
				   (pivots left) passes it, then stops on the line it
				   came in on.  Left/right stop on the first line. */
				turn_skip = (action == MOVE_UTURN) ? 1u : 0u;
				{
					b2_status_t s2;

					b2_get_status(&s2);
					turn_edges0 = s2.left_encoder_edges +
						      s2.right_encoder_edges;
				}
			} else if (result == B2_GOAL_FAILED) {
				state = LF_IDLE;
				tm_printf((UB*)"[S3] junction advance failed\n");
			}
		} else if (state == LF_TURN) {
			/* Sensor-only turn: no encoder angle, because the turn angle
			   from the provisional wheel calibration was ~1.7x too large
			   and carried the car past the line it wanted (2026-11-19
			   videos: U-turn -> turned 270 deg, right -> turned 180 deg).
			   Centre sensor state is debounced over S3_TURN_DEBOUNCE
			   loops.  First wait to leave the line the car is on, then
			   count each line the centre sensor reaches. */
			const BOOL timed_out =
			    ((uint32_t)(app_time_us_32() - turn_since) >=
			     (S3_TURN_MAX_MS * 1000u)) ? TRUE : FALSE;
			BOOL found = FALSE;
			BOOL past_min;
			{
				b2_status_t s2;

				/* Both wheels' edges (wrap-safe unsigned sum):
				   a pivot of A deg is 2 * A * ticks/deg. */
				b2_get_status(&s2);
				past_min = ((uint32_t)(s2.left_encoder_edges +
						       s2.right_encoder_edges -
						       turn_edges0) * 10u >=
					    2u * S3_TURN_MIN_DEG *
					    B2_CAL_TURN_TICKS_PER_DEG_X10) ? TRUE : FALSE;
			}

			if ((d.ir_center_raw ? TRUE : FALSE) != turn_center) {
				if (++turn_debounce >= S3_TURN_DEBOUNCE) {
					turn_center = !turn_center;
					turn_debounce = 0u;
					if (!turn_center) {
						turn_left_start = TRUE;
					} else if (turn_left_start && past_min) {
						if (turn_skip > 0u) {
							--turn_skip;
							tm_printf((UB*)"[S3] turn: passed a branch\n");
						} else {
							found = TRUE;
						}
					}
				}
			} else {
				turn_debounce = 0u;
			}

			if (found || timed_out) {
				state = LF_FOLLOW;
				lost_since = app_time_us_32();
				/* Found: if the pivot carries the car past the line, it
				   went too far the turn's way, so search back first.
				   Timed out: the normal search sweeps both ways. */
				line_is_left = (action == MOVE_RIGHT) ? TRUE : FALSE;
				tm_printf((UB*)"[S3] turn: %s after %d ms of pivot\n",
					  timed_out ? "line NOT found" : "line found",
					  (int)((uint32_t)(app_time_us_32() -
							   turn_since) / 1000u));
			} else {
				b2_command_t cmd = {0};
				const BOOL go_left = (action != MOVE_RIGHT) ? TRUE : FALSE;

				cmd.type = B2_CMD_SPEED;
				cmd.left_mm_s = go_left ? -S3_TURN_PIVOT_MM_S
							:  S3_TURN_PIVOT_MM_S;
				cmd.right_mm_s = -cmd.left_mm_s;
				(void)b2_submit_command(&cmd);
			}
		} else if (state == LF_FOLLOW) {
			if (d.line_detected) {
				lost_since = app_time_us_32();
			}

			/* One action per junction: re-arm only after leaving it. */
			if (!d.cross_junction) {
				on_junction = FALSE;
				junction_armed = TRUE;
			} else if (!on_junction) {
				on_junction = TRUE;
				junction_since = app_time_us_32();
			}
			if (on_junction && junction_armed &&
			    ((uint32_t)(app_time_us_32() - junction_since) >=
			     (S3_JUNCTION_CONFIRM_MS * 1000u))) {
				junction_armed = FALSE;
				action = get_action_for_character(stored);
				tm_printf((UB*)"[S3] junction, barcode '%c' -> action %d\n",
					  (stored != '\0') ? stored : '-', (int)action);
				stored = '\0';
				if ((action == MOVE_LEFT) || (action == MOVE_RIGHT) ||
				    (action == MOVE_UTURN)) {
					if (line_follow_goal(&goal, B2_CMD_MOVE_FORWARD,
					    S3_JUNCTION_ADVANCE_MM)) {
						state = LF_ADVANCE;
					} else {
						state = LF_IDLE;
						tm_printf((UB*)"[S3] junction advance refused\n");
					}
					s_lf.following = (state == LF_FOLLOW) ? 1U : 0U;
					(void)tk_dly_tsk(S3_FOLLOW_PERIOD_MS);
					continue;
				}
			}

			if ((uint32_t)(app_time_us_32() - lost_since) >=
			    (S3_LINE_LOST_MS * 1000u)) {
				b2_stop();
				state = LF_IDLE;
				tm_printf((UB*)"[S3] line lost, stopped\n");
			} else {
				line_follow_drive(&d, &line_is_left);
			}
		}

		s_lf.following = (state == LF_FOLLOW) ? 1U : 0U;
		(void)tk_dly_tsk(S3_FOLLOW_PERIOD_MS);
	}
}

#else /* !APP_MISSION_ENABLE */

EXPORT BOOL s3_line_following(void)
{
	return FALSE;	/* no line following in this build */
}

#endif /* APP_MISSION_ENABLE */

/* =====================================================================
 *  3. Subsystem glue (called from app_program/app_main.c)
 * ===================================================================== */

/* The barcode task.  Owns GP2, GP4 and GP16 (TEAM.md section 3). */
static void barcode_task(INT stacd, void *exinf)
{
	LineBarcodeData_t data;

	(void)stacd; (void)exinf;

	line_barcode_init();		/* set the pins up inside the owning task */

	while (1) {
		process_sensors_distance_packed(0, &data);

		if (data.scanned_code != S3_CODE_NONE) {
			char ch = s3_code_to_char(data.scanned_code);

			/* TMO_POL: never block here; drop the char if the
			   buffer is full rather than miss the next edges. */
			tk_snd_mbf(mbf_barcode, &ch, 1, TMO_POL);
			tm_printf((UB*)"[barcode] '%c'\n", ch);
		}

#if S3_POLL_DELAY_MS > 0
		tk_dly_tsk(S3_POLL_DELAY_MS);
#endif
	}
}

EXPORT ER subsystem3_init(void)
{
	T_CMBF	cmbf = {0};
	ER	er;

	/* Mutex that protects the driver's shared data */
	er = line_barcode_create();
	if (er < E_OK) {
		return er;
	}

	/* Message buffer: decoded characters, 1 byte each, a few queued */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 64;		/* room for a few queued chars (+ headers) */
	cmbf.maxmsz = 1;
	mbf_barcode = tk_cre_mbf(&cmbf);
	if (mbf_barcode < E_OK) {
		return mbf_barcode;
	}
#if APP_MISSION_ENABLE && S3_BENCH_MAPPING
	set_mqtt_action_mapping(S3_BENCH_FORWARD, S3_BENCH_UTURN,
				S3_BENCH_LEFT, S3_BENCH_RIGHT);
#endif
	return E_OK;
}

EXPORT ER subsystem3_start(void)
{
	/* Pinned to core 1 (TP_PRC2): the polling loop never sleeps, so keep
	   it away from core 0.  The pin is ignored on a single-core build. */
	ID tid = app_start_task(barcode_task, S3_PRI_MAIN, TP_PRC2,
				"barcode_task");
#if APP_MISSION_ENABLE
	if (tid >= E_OK) {
		/* Core 0: it sleeps between loops, and core 1 is kept busy. */
		tid = app_start_task(line_follow_task, S3_FOLLOW_PRI, TP_PRC1,
				     "line_follow_task");
	}
#endif
	return (tid < E_OK) ? tid : E_OK;
}