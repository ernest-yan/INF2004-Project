#include <stdio.h>
#include "pico/stdlib.h"
#include "line_barcode.h"

int main() {
    stdio_init_all();
    line_barcode_init();

    // Wait for USB Serial Monitor connection
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("\n===================================================\n");
    printf("  PICO W CODE 39 DECODER & LINE TRACKING TEST BENCH \n");
    printf("===================================================\n");
    printf("Instructions:\n");
    printf("1. Pass barcode paper over Right IR (GP16).\n");
    printf("2. Cover Left IR (GP2) or Center IR (GP4) to test line detection.\n\n");

    // Configure test MQTT mappings (Forward='F', U-Turn='U', Left='L', Right='R')
    set_mqtt_action_mapping('F', 'U', 'L', 'R');

    uint32_t simulated_encoder_ticks = 0;
    int8_t prev_line_state = -1;

    while (true) {
        simulated_encoder_ticks++;

        LineBarcodeData_t current_data;
        process_sensors_distance_packed(simulated_encoder_ticks, &current_data);

        // --- 1. Minimal Line Detection Check ---
        if (current_data.line_detected != prev_line_state) {
            if (current_data.line_detected == 1) {
                printf("line detected\n");
            } else {
                printf("out of track\n");
            }
            prev_line_state = current_data.line_detected;
        }

        // --- 2. Dynamic Barcode Output ---
        if (current_data.scanned_char != '\0') {
            char decoded = current_data.scanned_char;
            MovementAction_t action = get_action_for_character(decoded);

            printf("\n===================================================\n");
            printf(">>> BARCODE DECODED: '%c' <<<\n", decoded);
        }

        sleep_ms(1);
    }

    return 0;
}