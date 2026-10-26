#include "line_barcode.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <stdlib.h>

static LineBarcodeData_t g_system_data = {0};

// Dynamic MQTT Mapping Storage
static char g_mqtt_map_forward = '\0';
static char g_mqtt_map_uturn   = '\0';
static char g_mqtt_map_left    = '\0';
static char g_mqtt_map_right   = '\0';

typedef struct {
    char character;
    uint16_t pattern; // 9-bit Code 39 Pattern (5 bars, 4 spaces: 1 = Wide, 0 = Narrow)
} Code39Map;

// Complete A-Z Code 39 Standard Lookup Table + Start/Stop Asterisk Frame
static const Code39Map CODE39_TABLE[] = {
    {'*', 0b010010100}, // Code 39 Start/Stop Frame character
    {'A', 0b100001001}, {'B', 0b001001001}, {'C', 0b101001000}, {'D', 0b000011001},
    {'E', 0b100011000}, {'F', 0b001011000}, {'G', 0b000001101}, {'H', 0b100001100},
    {'I', 0b001001100}, {'J', 0b000011100}, {'K', 0b100000011}, {'L', 0b001000011},
    {'M', 0b101000010}, {'N', 0b000010011}, {'O', 0b100010010}, {'P', 0b001010010},
    {'Q', 0b000000111}, {'R', 0b100000110}, {'S', 0b001000110}, {'T', 0b000010110},
    {'U', 0b110000001}, {'V', 0b011000001}, {'W', 0b111000000}, {'X', 0b010010001},
    {'Y', 0b110010000}, {'Z', 0b011010000}
};

// Bit-reverses 9-bit integer for bidirectional (right-to-left) scanning
static uint16_t reverse_9bits(uint16_t pattern) {
    uint16_t rev = 0;
    for (int i = 0; i < 9; i++) {
        if (pattern & (1 << i)) {
            rev |= (1 << (8 - i));
        }
    }
    return rev;
}

// Configures movement mappings received over MQTT from PC
void set_mqtt_action_mapping(char char_fwd, char char_uturn, char char_left, char char_right) {
    uint32_t save = save_and_disable_interrupts();
    g_mqtt_map_forward = char_fwd;
    g_mqtt_map_uturn   = char_uturn;
    g_mqtt_map_left    = char_left;
    g_mqtt_map_right   = char_right;
    restore_interrupts(save);
}

// Maps decoded character to requested movement action
MovementAction_t get_action_for_character(char c) {
    if (c == '\0') return MOVE_NONE;
    if (c == g_mqtt_map_forward) return MOVE_FORWARD;
    if (c == g_mqtt_map_uturn)   return MOVE_UTURN;
    if (c == g_mqtt_map_left)    return MOVE_LEFT;
    if (c == g_mqtt_map_right)   return MOVE_RIGHT;
    return MOVE_NONE;
}

void line_barcode_init(void) {
    gpio_init(PIN_IR_LEFT);   gpio_set_dir(PIN_IR_LEFT, GPIO_IN);
    gpio_init(PIN_IR_CENTER); gpio_set_dir(PIN_IR_CENTER, GPIO_IN);
    gpio_init(PIN_IR_RIGHT);  gpio_set_dir(PIN_IR_RIGHT, GPIO_IN);
}

LineBarcodeData_t get_line_barcode_data_safe(void) {
    uint32_t save = save_and_disable_interrupts();
    LineBarcodeData_t copy = g_system_data;
    restore_interrupts(save);
    return copy;
}

void process_sensors_distance_packed(uint32_t current_ticks, LineBarcodeData_t *out_data) {
    static uint32_t widths[MAX_BARS];
    static uint8_t bar_count = 0;
    static uint32_t last_edge_time = 0;
    static bool last_state = false;

    bool ir_left   = gpio_get(PIN_IR_LEFT);    // Grove 2 (GP2)
    bool ir_center = gpio_get(PIN_IR_CENTER); // Grove 3 (GP4)
    bool ir_right  = gpio_get(PIN_IR_RIGHT);  // Grove 4 (GP16)

    uint32_t now_us = time_us_32();
    char decoded_char = '\0';

    // --- 1. Right IR: ST188 Barcode Pulse Timing ---
    if (ir_right != last_state) {
        uint32_t duration_us = now_us - last_edge_time;
        
        // Debounce filter: Ignore noise spikes shorter than 500us
        if (duration_us > 500 || bar_count == 0) {
            if (bar_count < MAX_BARS && last_edge_time != 0) {
                widths[bar_count++] = duration_us;
            }
            last_edge_time = now_us;
            last_state = ir_right;
        }
    }

    uint32_t time_since_last_edge = now_us - last_edge_time;

    // --- 2. Code 39 Evaluation on Quiet Timeout (50ms) ---
    static uint32_t last_success_decode_time = 0;

    if (bar_count >= 9 && (time_since_last_edge > 50000)) {
        
        // 300ms Cooldown lockout to prevent duplicate reads during single sweep
        if (now_us - last_success_decode_time < 300000) {
            bar_count = 0;
            return;
        }

        uint32_t temp[9];
        for (int i = 0; i < 9; i++) {
            temp[i] = widths[i];
        }

        // Bubble sort to extract narrow vs wide element durations
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8 - i; j++) {
                if (temp[j] > temp[j + 1]) {
                    uint32_t t = temp[j];
                    temp[j] = temp[j + 1];
                    temp[j + 1] = t;
                }
            }
        }

        uint32_t max_narrow = temp[5];
        uint32_t min_wide   = temp[6];

        // Safety Guard: Verify Code 39 3:1 Wide-to-Narrow Ratio
        if (min_wide < (uint32_t)(max_narrow * 1.3f)) {
            bar_count = 0;
            return;
        }

        uint32_t threshold = max_narrow + ((min_wide - max_narrow) / 2);

        // Build 9-bit binary pattern
        uint16_t pattern = 0;
        for (int i = 0; i < 9; i++) {
            if (widths[i] > threshold) {
                pattern |= (1 << (8 - i));
            }
        }

        uint16_t rev_pattern = reverse_9bits(pattern);

        // Match against Code 39 Table (Forward OR Reverse)
        for (size_t i = 0; i < sizeof(CODE39_TABLE)/sizeof(CODE39_TABLE[0]); i++) {
            if (CODE39_TABLE[i].pattern == pattern || CODE39_TABLE[i].pattern == rev_pattern) {
                if (CODE39_TABLE[i].character != '*') { // Skip framing asterisks
                    decoded_char = CODE39_TABLE[i].character;
                    last_success_decode_time = now_us;
                    break;
                }
            }
        }

        bar_count = 0;
    } else if (time_since_last_edge > 50000) {
        bar_count = 0;
    }

    // --- 3. Left & Center IR: Line Tracking & Cross-Junction Detection ---
    uint8_t line_state  = (ir_left || ir_center) ? 1 : 0;
    uint8_t cross_state = (ir_left && ir_center) ? 1 : 0; // Cross junction trigger

    // --- 4. Atomic Bit-Packed Application State Save ---
    uint32_t save = save_and_disable_interrupts();
    g_system_data.ir_left_raw    = ir_left;
    g_system_data.ir_center_raw  = ir_center;
    g_system_data.ir_right_raw   = ir_right;
    g_system_data.line_detected  = line_state;
    g_system_data.cross_junction = cross_state;
    g_system_data.scanned_char   = decoded_char;

    if (!line_state) {
        g_system_data.fault_mask |= FAULT_LINE_LOSS;
    } else {
        g_system_data.fault_mask &= ~FAULT_LINE_LOSS;
    }

    if (out_data) {
        *out_data = g_system_data;
    }

    g_system_data.scanned_char = '\0'; // Auto-clear read buffer
    restore_interrupts(save);
}