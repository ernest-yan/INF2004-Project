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

#define FAULT_NONE      0x00
#define FAULT_LINE_LOSS (1u << 0)

typedef enum {
    MOVE_NONE = 0,
    MOVE_FORWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_UTURN
} MovementAction_t;

/*
 * Compact public data structure from the original project.
 * scanned_char is set for ONE process call when a new character is decoded.
 */
typedef struct __attribute__((packed)) {
    uint8_t line_detected  : 1;
    uint8_t cross_junction : 1;
    uint8_t ir_left_raw    : 1;
    uint8_t ir_center_raw  : 1;
    uint8_t ir_right_raw   : 1;
    uint8_t reserved       : 3;
    char scanned_char;
    uint8_t fault_mask;
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
 * Line following + barcode turns (line_follow_task, APP_MISSION_ENABLE only).
 * PROVISIONAL values: tune on the real track, wheels lifted first.
 */
#define S3_FOLLOW_PRI		17	/* priority band 16-17 (TEAM.md) */
#define S3_FOLLOW_PERIOD_MS	10
#define S3_LINE_SPEED_MM_S	100	/* both wheels, on the line */
#define S3_STEER_SPEED_MM_S	90	/* outer wheel while steering (inner = 0) */
#define S3_TURN_SPEED_MM_S	80	/* barcode turns */
#define S3_TURN_TIMEOUT_MS	6000
#define S3_JUNCTION_CONFIRM_MS	40	/* both sensors on the line this long */
#define S3_JUNCTION_ADVANCE_MM	60	/* line sensors to wheel axle: measure */
#define S3_LINE_LOST_MS		1500	/* search this long, then stop */
#define S3_START_DELAY_MS	3000	/* after boot, before the first move */

/*
 * The real letter -> movement mapping arrives over MQTT at the demo through
 * set_mqtt_action_mapping() (subsystem 1).  Until that exists this bench
 * mapping lets the turns be tested.  MQTT replaces it as soon as it arrives.
 * Set to 0 once subsystem 1 is in: the mapping must not be hard-coded.
 */
#define S3_BENCH_MAPPING	1
#define S3_BENCH_FORWARD	'F'
#define S3_BENCH_UTURN		'U'
#define S3_BENCH_LEFT		'L'
#define S3_BENCH_RIGHT		'R'

/* IF-01: S3 -> any subsystem.  Message buffer, one byte per decoded Code 39
   character ('A'..'Z').  Receive with tk_rcv_mbf(mbf_barcode, &c, TMO_FEVR). */
IMPORT ID	mbf_barcode;

#endif /* LINE_BARCODE_H */
