/*
 *	line_barcode.h
 *	Subsystem 3 -- line sensors + Code 39 barcode decoder (driver AND
 *	subsystem glue in one pair of files).
 *
 *	Ported from the bare-metal Pico SDK version.  Differences:
 *	  - no Pico SDK headers: GPIO goes through libbsp (gpio_set_pin / gpio_get_val)
 *	  - interrupt masking replaced by a kernel mutex
 *	  - line_barcode_create() must be called once from subsystem3_init()
 */

#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

/*
 * Cytron Maker Board Grove connections:
 * Grove 2: GP2  -> left line sensor
 * Grove 3: GP4  -> center line sensor
 * Grove 4: GP16 -> right barcode sensor
 * (claim these pins in TEAM.md section 2)
 */
#define PIN_IR_LEFT   2
#define PIN_IR_CENTER 4
#define PIN_IR_RIGHT  16

/*
 * TIMEOUT_US: time without an edge before the collected scan is finalized.
 * It must be longer than the widest bar takes to pass the sensor at the
 * slowest driving speed: a 9 mm wide bar (141 mm barcode) at the 45 mm/s of a
 * steering car takes 200 ms.
 */
#define MAX_BARS       60
#define TIMEOUT_US     400000u
#define MIN_EDGE_US    100u
#define DECODE_COOLDOWN_US 300000u

/*
 * A finished GP16 scan is decoded (and reported to S1) once it has at least
 * this many elements, the idle gap before the first bar included.  5 = 4
 * bars/spaces: crossing the track line (1 bar) never counts as a scan, but a
 * barcode the sensor only partly caught is still reported (was 9).
 */
#define S3_SCAN_REPORT_MIN 5u

/* Outcome of the newest finished scan (LineBarcodeData_t.last_scan_result,
   sent to the dashboard in the BARCODE event). */
#define S3_SCAN_OK         0u	/* a letter was decoded */
#define S3_SCAN_SHORT      1u	/* fewer elements than a full barcode (29) */
#define S3_SCAN_NO_FRAME   2u	/* enough elements, but no '*' start AND
				   stop character found at any offset */
#define S3_SCAN_BAD_LETTER 3u	/* '*' frame found, but the middle nine are
				   not a Code 39 letter (bar widths off) */

#define FAULT_NONE      0x00
#define FAULT_LINE_LOSS (1u << 0)

typedef enum {
    MOVE_NONE = 0,
    MOVE_FORWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_UTURN
} MovementAction_t;

/* CR1: 6-bit character code for barcode letters and the MQTT map letters:
 * 0 = none, 1..26 = 'A'..'Z', 27..36 = '0'..'9' (37 values). */
#define S3_CODE_NONE    (0U)
#define S3_CODE_BITS    (6U)

static inline uint8_t s3_char_to_code(char c)
{
    if ((c >= 'A') && (c <= 'Z')) { return (uint8_t)((uint8_t)(c - 'A') + 1U); }
    if ((c >= '0') && (c <= '9')) { return (uint8_t)((uint8_t)(c - '0') + 27U); }
    return S3_CODE_NONE;
}

static inline char s3_code_to_char(uint32_t code)
{
    if ((code >= 1U) && (code <= 26U))  { return (char)('A' + (code - 1U)); }
    if ((code >= 27U) && (code <= 36U)) { return (char)('0' + (code - 27U)); }
    return '\0';
}

/*
 * Compact public data structure from the original project.
 * scanned_code is set for ONE process call when a new character is decoded.
 * S1: barcode_count / last_code keep it for readers that poll slowly
 * (S1 telemetry at 5 Hz), without taking it from mbf_barcode.
 * CR1: 6 -> 4 bytes. Read the letters with s3_code_to_char().
 * 2026-11-19: 4 -> 5 bytes for the scan diagnosis (last_scan_*), so the
 * dashboard can say WHY a barcode was not decoded.
 */
typedef struct __attribute__((packed)) {
    uint8_t line_detected  : 1;
    uint8_t cross_junction : 1;
    uint8_t ir_left_raw    : 1;
    uint8_t ir_center_raw  : 1;
    uint8_t ir_right_raw   : 1;
    uint8_t fault_mask     : 1; /* bit 0 = FAULT_LINE_LOSS (only fault) */
    uint8_t scan_fail_count : 2; /* S1: +1 per scan of S3_SCAN_REPORT_MIN+
                                    elements that gave no letter (barcode
                                    SEEN, not decoded); wraps. Scans end
                                    >= 400 ms apart and S1 reads every
                                    200 ms, so 2 bits never miss. */
    uint8_t scanned_code   : 6; /* S3_CODE, this call only */
    uint8_t                : 2;
    uint8_t last_code      : 6; /* S1: newest decoded letter, NONE = none yet */
    uint8_t                : 2;
    uint8_t barcode_count;      /* S1: +1 per decoded character, wraps */
    uint8_t last_scan_elems  : 6; /* S1: bars + spaces in the newest finished
                                     scan (decoded or not); a full barcode
                                     is 29 */
    uint8_t last_scan_result : 2; /* S1: S3_SCAN_OK / _SHORT / _NO_FRAME /
                                     _BAD_LETTER for that scan */
} LineBarcodeData_t;

/* Create the kernel mutex that protects the shared data.  Call ONCE from
   subsystem3_init().  Returns E_OK or a negative kernel error code. */
ER line_barcode_create(void);

/* Configure the three sensor pins.  Call from the task that owns them. */
void line_barcode_init(void);

LineBarcodeData_t get_line_barcode_data_safe(void);
void process_sensors_distance_packed(uint32_t current_ticks,
                                     LineBarcodeData_t *out_data);

void set_mqtt_action_mapping(char char_fwd,
                             char char_uturn,
                             char char_left,
                             char char_right);
MovementAction_t get_action_for_character(char c);

/* ---------------------------------------------------------------------
 *  Subsystem 3 (public interface)
 * --------------------------------------------------------------------- */
#define S3_PRI_MAIN		16	/* priority band 16-17 (TEAM.md) */

/*
 * Poll delay of the barcode task in ms.  0 = poll continuously (needed
 * because bar widths are measured in microseconds and the kernel tick is
 * 1 ms).  If you set this above 0, narrow bars are mis-measured.
 */
#define S3_POLL_DELAY_MS	0

/*
 * TEMPORARY bench debug.  1 = print every finished GP16 scan (element count
 * and widths in us) and, while idle, the sensor levels every 2 s, so a scan
 * that does not decode can be diagnosed.  The demo must use 0.
 */
#define S3_DEBUG_SCAN		0

/*
 * GP16 must hold a new level this long before it counts as a bar edge.
 * 0 = off: every edge counts, exactly as the original decoder (this is the
 * setting that decoded 'F' on the bench, 2026-11-13).  8000 was tried and
 * merged real narrow bars at hand speed (~7 ms), so keep it 0 unless the car
 * shows edge flicker at driving speed, and then stay well below the
 * narrowest bar time.
 */
#define S3_EDGE_DEBOUNCE_US	0u

/*
 * Line following + barcode turns (line_follow_task, APP_MISSION_ENABLE only).
 * PROVISIONAL values: tune on the real track, wheels lifted first.
 */
#define S3_FOLLOW_PRI		17	/* priority band 16-17 (TEAM.md) */
#define S3_FOLLOW_PERIOD_MS	10
#define S3_LINE_SPEED_MM_S	45	/* both wheels, on the line (100 lost the
					   line within 0.3 s, 2026-11-16 telemetry;
					   70 wobbled on the S-curves, 2026-11-19
					   video; 55 gave short/unclear barcode
					   scans, 2026-11-19 dashboard).  Keep
					   >= 40: S2 stall check */
#define S3_STEER_SPEED_MM_S	70	/* outer wheel while steering (90 ran
					   at 115-134 mm/s and overshot) */
#define S3_STEER_INNER_MM_S	30	/* inner wheel while steering: a curve, not a
					   pivot (inner = 0 overshot the line, 2026-11-13) */
#define S3_STEER_HARD_MS	400	/* still steering the same way after this:
					   hard turn (tight curves). Was 250 */
#define S3_STEER_HARD_MM_S	0	/* reversed inner wheel speed in the hard
					   turn. 0 = inner wheel stops, so the car
					   turns about that wheel and keeps moving
					   into the corner. Was 40 (reverse): the car
					   spun on the spot before its wheels reached
					   the 90 deg corner and ended up parallel to,
					   but beside, the new line (2026-11-19 video).
					   S2 checks for stalls only from 40 mm/s up */
#define S3_STEER_EXIT_MS	0	/* centre found the line again after a
					   correction: keep arcing gently that way
					   this long (0 = off, snap straight).
					   Try 150 only if the car undershoots
					   curves (wobbles on the inside) */
#define S3_STEER_EXIT_INNER_MM_S 50	/* inner wheel during that soft exit
					   (outer = S3_LINE_SPEED_MM_S) */
#define S3_SCAN_HOLD_MAX_MS	4500	/* drive straight at most this long while
					   GP16 scans (141 mm barcode = 3.1 s at
					   45; raise it if the line speed drops) */
#define S3_TURN_SPEED_MM_S	80	/* barcode turns */
#define S3_TURN_TIMEOUT_MS	6000
/* A barcode turn pivots and counts lines under the centre sensor; it uses
   no encoder angle.  Encoder turns of 90/180 deg (and later a blind
   55/145 deg part) went past the wanted line: the real turn was ~1.7x the
   commanded one with the provisional wheel calibration (2026-11-19 videos
   detected_barcode_turn_overturn.mp4, IMG_2884.mp4). */
#define S3_TURN_PIVOT_MM_S	45	/* each wheel, opposite ways (~75 deg/s
					   measured on video). >= 40: S2's stall
					   check stays active */
#define S3_TURN_DEBOUNCE	3	/* loops (x S3_FOLLOW_PERIOD_MS) the centre
					   sensor must agree before a line edge
					   counts: tile seams and dirt ignored */
#define S3_TURN_MIN_DEG		25	/* ignore lines until the encoders say the
					   car has turned this far: the centre
					   sensor can end the advance beside the
					   line it is on and "find" that line again
					   after a few degrees (2026-11-19 20:51 log:
					   2 of 5 right turns ended after 0.15 s and
					   went straight).  Only a loose gate: the
					   encoder angle reads ~0.6x the real one
					   with the provisional calibration, and the
					   new line is at 90 */
#define S3_TURN_MAX_MS		6000	/* no line found by then (a U-turn needs
					   ~2.5 s): hand over to the normal search */
#define S3_JUNCTION_CONFIRM_MS	40	/* both sensors on the line this long */
#define S3_JUNCTION_ADVANCE_MM	60	/* line sensors to wheel axle: measure */
#define S3_LINE_LOST_MS		2500	/* search this long, then stop */
#define S3_SEARCH_SWAP_MS	300	/* lost: guessed side this long, then the
					   other side for the rest of the search */
#define S3_START_DELAY_MS	3000	/* after boot, before the first move */

/*
 * The real letter -> movement mapping arrives over MQTT at the demo through
 * set_mqtt_action_mapping() (subsystem 1, dashboard "map FULR").  Until it
 * arrives every barcode means MOVE_NONE (go straight on).
 * 1 = hard-code the bench mapping below at boot (bench tests without WiFi
 * only; MQTT still replaces it).  The demo must use 0.
 */
#define S3_BENCH_MAPPING	0
#define S3_BENCH_FORWARD	'F'
#define S3_BENCH_UTURN		'U'
#define S3_BENCH_LEFT		'L'
#define S3_BENCH_RIGHT		'R'

/* IF-01: S3 -> any subsystem.  Message buffer, one byte per decoded Code 39
   character ('A'..'Z').  Receive with tk_rcv_mbf(mbf_barcode, &c, TMO_FEVR). */
IMPORT ID	mbf_barcode;

/* TRUE only while line_follow_task is plainly following the line (not
   idle, not driving onto a junction, not turning).  S5 only reacts to
   obstacles while this is TRUE, so a parked car never starts a bypass and a
   junction move is never cut short.  Always FALSE without APP_MISSION_ENABLE. */
IMPORT BOOL	s3_line_following(void);

#endif /* LINE_BARCODE_H */
