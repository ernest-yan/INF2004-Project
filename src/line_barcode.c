#include "line_barcode.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * Code 39 patterns are represented as nine elements in scan order:
 * 1 = wide, 0 = narrow. The standard has 3 wide elements per character.
 *
 * These are the same A-Z patterns used by the original project.
 * '*' is the Code 39 start/stop character; it is recognized but not emitted.
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
    uint32_t save = save_and_disable_interrupts();
    g_mqtt_map_forward = char_fwd;
    g_mqtt_map_uturn   = char_uturn;
    g_mqtt_map_left    = char_left;
    g_mqtt_map_right   = char_right;
    restore_interrupts(save);
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
    gpio_init(PIN_IR_LEFT);
    gpio_set_dir(PIN_IR_LEFT, GPIO_IN);

    gpio_init(PIN_IR_CENTER);
    gpio_set_dir(PIN_IR_CENTER, GPIO_IN);

    gpio_init(PIN_IR_RIGHT);
    gpio_set_dir(PIN_IR_RIGHT, GPIO_IN);

    g_width_count = 0;
    g_last_edge_time = 0;
    g_sensor_state_initialized = false;
    g_have_successful_decode = false;
}

LineBarcodeData_t get_line_barcode_data_safe(void)
{
    uint32_t save = save_and_disable_interrupts();
    LineBarcodeData_t copy = g_system_data;
    restore_interrupts(save);
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

static char lookup_code39(uint16_t pattern)
{
    uint16_t reversed = reverse_9bits(pattern);

    for (size_t i = 0; i < sizeof(CODE39_TABLE) / sizeof(CODE39_TABLE[0]); ++i) {
        if (CODE39_TABLE[i].pattern == pattern ||
            CODE39_TABLE[i].pattern == reversed) {
            /* Start/stop '*' is framing, not an application character. */
            return CODE39_TABLE[i].character == '*' ? '\0'
                                                     : CODE39_TABLE[i].character;
        }
    }

    return '\0';
}

/*
 * Try every contiguous nine-duration window in the collected scan.
 * This helps when the first edge occurs partway through a character.
 * Returns the first non-framing Code 39 character found.
 */
static char decode_collected_scan(void)
{
    if (g_width_count < 9) return '\0';

    for (uint8_t start = 0; (uint16_t)start + 9u <= g_width_count; ++start) {
        uint32_t candidate[9];

        for (int i = 0; i < 9; ++i) {
            candidate[i] = g_widths[start + i];
        }

        uint16_t pattern;
        if (!classify_nine(candidate, &pattern)) {
            continue;
        }

        char c = lookup_code39(pattern);
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

    bool ir_left = gpio_get(PIN_IR_LEFT);
    bool ir_center = gpio_get(PIN_IR_CENTER);
    bool ir_right = gpio_get(PIN_IR_RIGHT);

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

    uint32_t save = save_and_disable_interrupts();

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

    restore_interrupts(save);
}
