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
 * Start at 50000 us. If separate characters are merged, reduce it;
 * if a scan is cut off too early, increase it.
 */
#define MAX_BARS       60
#define TIMEOUT_US     50000u
#define MIN_EDGE_US    250u
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

/* IF-01: S3 -> any subsystem.  Message buffer, one byte per decoded Code 39
   character ('A'..'Z').  Receive with tk_rcv_mbf(mbf_barcode, &c, TMO_FEVR). */
IMPORT ID	mbf_barcode;

#endif /* LINE_BARCODE_H */
