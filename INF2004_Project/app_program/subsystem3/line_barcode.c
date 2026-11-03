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
#include "../subsystem4/tkernel_adapter.h"
#include "../subsystem5/ultrasonic.h"

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

static char g_mqtt_map_forward = '\0';
static char g_mqtt_map_uturn   = '\0';
static char g_mqtt_map_left    = '\0';
static char g_mqtt_map_right   = '\0';

/* One duration per completed HIGH/LOW element. */
static uint32_t g_widths[MAX_BARS];
static uint8_t g_width_count = 0;
static uint32_t g_last_edge_time = 0;
static bool g_last_sensor_state = false;
static bool g_sensor_state_initialized = false;
static uint32_t g_last_success_decode_time = 0;
static bool g_have_successful_decode = false;

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
    data_lock();	/* PORT: was save_and_disable_interrupts() */
    g_mqtt_map_forward = char_fwd;
    g_mqtt_map_uturn   = char_uturn;
    g_mqtt_map_left    = char_left;
    g_mqtt_map_right   = char_right;
    data_unlock();	/* PORT: was restore_interrupts() */
}

MovementAction_t get_action_for_character(char c)
{
    if (c == '\0') return MOVE_NONE;

    if (g_mqtt_map_forward != '\0' && c == g_mqtt_map_forward)
        return MOVE_FORWARD;
    if (g_mqtt_map_uturn != '\0' && c == g_mqtt_map_uturn)
        return MOVE_UTURN;
    if (g_mqtt_map_left != '\0' && c == g_mqtt_map_left)
        return MOVE_LEFT;
    if (g_mqtt_map_right != '\0' && c == g_mqtt_map_right)
        return MOVE_RIGHT;

    return MOVE_NONE;
}

void line_barcode_init(void)
{
    /* PORT: gpio_init() + gpio_set_dir(GPIO_IN) -> gpio_set_pin(GPIO_MODE_IN) */
    gpio_set_pin(PIN_IR_LEFT,   GPIO_MODE_IN);
    gpio_set_pin(PIN_IR_CENTER, GPIO_MODE_IN);
    gpio_set_pin(PIN_IR_RIGHT,  GPIO_MODE_IN);

    g_width_count = 0;
    g_last_edge_time = 0;
    g_sensor_state_initialized = false;
    g_have_successful_decode = false;
}

LineBarcodeData_t get_line_barcode_data_safe(void)
{
    data_lock();	/* PORT */
    LineBarcodeData_t copy = g_system_data;
    data_unlock();	/* PORT */
    return copy;
}

/*
 * Sort nine durations, estimate a narrow/wide boundary from the gap between
 * the 6th and 7th sorted values, and classify the original ordered durations.
 *
 * A candidate is rejected if there is no separation between narrow and wide
 * durations, or if the result does not have exactly three wide elements.
 */
static bool classify_nine(const uint32_t in[9], uint16_t *pattern_out)
{
    uint32_t sorted[9];

    for (int i = 0; i < 9; ++i) {
        if (in[i] == 0) return false;
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
        candidate[i] = g_widths[start + i];
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
static char decode_collected_scan(void)
{
    if (g_width_count < CODE39_FRAME_ELEMENTS) return '\0';

    for (uint8_t start = 0;
         (uint16_t)start + CODE39_FRAME_ELEMENTS <= g_width_count; ++start) {
        uint16_t first, letter, last;

        if (!classify_at(start, &first) ||
            !classify_at((uint8_t)(start + 20u), &last) ||
            !classify_at((uint8_t)(start + 10u), &letter)) {
            continue;
        }

        if (first != CODE39_STAR_PATTERN || last != CODE39_STAR_PATTERN) {
            if (reverse_9bits(first) != CODE39_STAR_PATTERN ||
                reverse_9bits(last) != CODE39_STAR_PATTERN) {
                continue;
            }
            letter = reverse_9bits(letter);
        }

        char c = lookup_code39(letter);
        if (c != '\0') {
            return c;
        }
    }

    return '\0';
}

static void append_duration(uint32_t duration)
{
    if (duration < MIN_EDGE_US) {
        return; /* Ignore very short noise pulses. */
    }

    if (g_width_count < MAX_BARS) {
        g_widths[g_width_count++] = duration;
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

    /*
     * Seed the edge timer from the first sample. Do not treat startup time as
     * the duration of the first barcode element.
     */
    if (!g_sensor_state_initialized) {
        g_last_sensor_state = ir_right;
        g_last_edge_time = now_us;
        g_sensor_state_initialized = true;
    } else if (ir_right != g_last_sensor_state) {
        uint32_t duration = now_us - g_last_edge_time;
        append_duration(duration);
        g_last_edge_time = now_us;
        g_last_sensor_state = ir_right;
    }

    uint32_t quiet_time = now_us - g_last_edge_time;

    /*
     * Finalize when no transition has occurred for TIMEOUT_US. Decode from the
     * collected edge durations and then clear the scan buffer.
     */
    if (g_width_count >= 9 && quiet_time >= TIMEOUT_US) {
        bool cooldown_over = !g_have_successful_decode ||
            (uint32_t)(now_us - g_last_success_decode_time) >=
                DECODE_COOLDOWN_US;

        if (cooldown_over) {
            decoded_char = decode_collected_scan();
            if (decoded_char != '\0') {
                g_last_success_decode_time = now_us;
                g_have_successful_decode = true;
            }
        }

        g_width_count = 0;
    } else if (quiet_time >= TIMEOUT_US && g_width_count > 0) {
        /* Incomplete scan; discard it after the quiet timeout. */
        g_width_count = 0;
    }

    /* Line tracking: retain the original active-HIGH logic. */
    uint8_t line_state = (ir_left || ir_center) ? 1u : 0u;
    uint8_t cross_state = (ir_left && ir_center) ? 1u : 0u;

    data_lock();	/* PORT: was save_and_disable_interrupts() */

    g_system_data.ir_left_raw = ir_left ? 1u : 0u;
    g_system_data.ir_center_raw = ir_center ? 1u : 0u;
    g_system_data.ir_right_raw = ir_right ? 1u : 0u;
    g_system_data.line_detected = line_state;
    g_system_data.cross_junction = cross_state;
    g_system_data.scanned_char = decoded_char;

    if (!line_state) {
        g_system_data.fault_mask |= FAULT_LINE_LOSS;
    } else {
        g_system_data.fault_mask &= (uint8_t)~FAULT_LINE_LOSS;
    }

    if (out_data != NULL) {
        *out_data = g_system_data;
    }

    /* One-cycle event: the caller receives the character through out_data. */
    g_system_data.scanned_char = '\0';

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
 *    neither         keep steering to the side the line was last seen on
 *                    (from centre-only that is right: the left sensor would
 *                    have seen it otherwise), give up after S3_LINE_LOST_MS
 *
 *  A decoded barcode is only stored.  Its movement runs at the next
 *  cross-junction (both sensors on the line), after driving on until the
 *  wheels are over the junction.  Then the stored letter is cleared.
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
	LF_TURN		/* the turn for the stored barcode is running */
} line_follow_state_t;

/* TRUE when S3 may drive: no S5 obstacle handling, no latched S2 brake. */
static BOOL line_follow_allowed(void)
{
	subsystem5_status_t s5;
	b2_status_t s2;

	subsystem5_get_status(&s5);
	b2_get_status(&s2);
	return (s5.state == subsystem5_STATE_MONITOR) && (s5.stop_required == 0U) &&
	       (s2.brake_latched == 0U) && (s2.state != B2_FAULT);
}

static void line_follow_drive(const LineBarcodeData_t *d, BOOL *line_is_left)
{
	b2_command_t cmd = {0};

	cmd.type = B2_CMD_SPEED;
	if (d->ir_center_raw) {
		*line_is_left = FALSE;
		cmd.left_mm_s = S3_LINE_SPEED_MM_S;
		cmd.right_mm_s = S3_LINE_SPEED_MM_S;
	} else {
		if (d->ir_left_raw) {
			*line_is_left = TRUE;
		}
		cmd.left_mm_s = *line_is_left ? 0 : S3_STEER_SPEED_MM_S;
		cmd.right_mm_s = *line_is_left ? S3_STEER_SPEED_MM_S : 0;
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
	uint32_t junction_since = 0u;
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
			if (state != LF_IDLE) {
				b2_stop();
				state = LF_IDLE;
				tm_printf((UB*)"[S3] holding: S5 or S2 needs the wheels stopped\n");
			}
		} else if (state == LF_IDLE) {
			if (d.line_detected) {
				state = LF_FOLLOW;
				lost_since = app_time_us_32();
				tm_printf((UB*)"[S3] line found, following\n");
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
				if (line_follow_goal(&goal,
				    (action == MOVE_RIGHT) ? B2_CMD_TURN_RIGHT : B2_CMD_TURN_LEFT,
				    (action == MOVE_UTURN) ? 180U : 90U)) {
					state = LF_TURN;
				} else {
					state = LF_IDLE;
					tm_printf((UB*)"[S3] junction turn refused\n");
				}
			} else if (result == B2_GOAL_FAILED) {
				state = LF_IDLE;
				tm_printf((UB*)"[S3] junction advance failed\n");
			}
		} else if (state == LF_TURN) {
			const b2_goal_result_t result = b2_goal_check(&goal);

			if (result == B2_GOAL_DONE) {
				state = LF_FOLLOW;
				lost_since = app_time_us_32();
			} else if (result == B2_GOAL_FAILED) {
				state = LF_IDLE;
				tm_printf((UB*)"[S3] junction turn failed\n");
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

		(void)tk_dly_tsk(S3_FOLLOW_PERIOD_MS);
	}
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

		if (data.scanned_char != '\0') {
			char ch = data.scanned_char;

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