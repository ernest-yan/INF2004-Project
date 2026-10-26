#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include <stdint.h>
#include <stdbool.h>

// Cytron Board Grove Port Pin Definitions
#define PIN_IR_LEFT   2  // Grove 2 Port -> GP2 (Left Line Sensor)
#define PIN_IR_CENTER 4  // Grove 3 Port -> GP4 (Center Line Sensor)
#define PIN_IR_RIGHT  16 // Grove 4 Port -> GP16 (Right Barcode Sensor)

#define MAX_BARS 30
#define TIMEOUT_US 50000 // 50ms quiet timeout tuned for Code 39 sweeps

// System fault bitmask definitions
#define FAULT_NONE          0x00
#define FAULT_LINE_LOSS     (1 << 0) // Bit 0: Line lost

typedef enum {
    MOVE_NONE = 0,
    MOVE_FORWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_UTURN
} MovementAction_t;

// Minimum-Bit Packed Application Data Structure
typedef struct __attribute__((packed)) {
    uint8_t line_detected  : 1; // 1 = Line Detected (Left or Center), 0 = Out of Track
    uint8_t cross_junction : 1; // 1 = Cross-Junction Detected (Left AND Center)
    uint8_t ir_left_raw    : 1; // Raw GP2 state
    uint8_t ir_center_raw  : 1; // Raw GP4 state
    uint8_t ir_right_raw   : 1; // Raw GP16 state
    uint8_t reserved       : 3; // Alignment padding
    char    scanned_char;       // Decoded ASCII character ('A' - 'Z')
    uint8_t fault_mask;         // Shared fault bitmask
} LineBarcodeData_t;

void line_barcode_init(void);
LineBarcodeData_t get_line_barcode_data_safe(void);
void process_sensors_distance_packed(uint32_t current_ticks, LineBarcodeData_t *out_data);

// Dynamic MQTT Mapping Functions
void set_mqtt_action_mapping(char char_fwd, char char_uturn, char char_left, char char_right);
MovementAction_t get_action_for_character(char c);

#endif // LINE_BARCODE_H