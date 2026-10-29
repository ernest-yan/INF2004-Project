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
    printf("   PICO W CODE 39 DECODER & LINE TRACKING TEST BENCH \n");
    printf("===================================================\n");
    printf("Instructions:\n");
    printf("1. Pass barcode paper over Right IR (GP16).\n");
    printf("2. Cover Left IR (GP2) or Center IR (GP4) for line detection.\n");
    printf("3. Cover BOTH Left (GP2) and Center (GP4) for cross-junction.\n\n");

    // Configure test MQTT mappings (Forward='F', U-Turn='U', Left='L', Right='R')
    set_mqtt_action_mapping('F', 'U', 'L', 'R');

    uint32_t simulated_encoder_ticks = 0;
    int8_t prev_line_state = -1;
    uint8_t prev_cross_state = 0;

    // Read initial state to prevent immediate false state-change trigger on boot
    LineBarcodeData_t init_data;
    process_sensors_distance_packed(0, &init_data);
    prev_line_state = init_data.line_detected;
    prev_cross_state = init_data.cross_junction;

    printf("[SYSTEM READY] Initial Line State: %s\n\n", 
           (prev_line_state == 1) ? "LINE DETECTED" : "OUT OF TRACK");

    while (true) {
        simulated_encoder_ticks++;

        LineBarcodeData_t current_data;
        process_sensors_distance_packed(simulated_encoder_ticks, &current_data);

        // --- 1. Line Detection State Transitions ---
        if (current_data.line_detected != prev_line_state) {
            if (current_data.line_detected == 1) {
                printf("[TRACK] Line detected\n");
            } else {
                printf("[TRACK] Out of track\n");
            }
            prev_line_state = current_data.line_detected;
        }

        // --- 2. Cross-Junction Detection ---
        if (current_data.cross_junction != prev_cross_state) {
            if (current_data.cross_junction == 1) {
                printf("\n===================================================\n");
                printf(">>> CROSS JUNCTION DETECTED <<<\n");
                printf("===================================================\n\n");
            }
            prev_cross_state = current_data.cross_junction;
        }

        // --- 3. Barcode Decode Output & Action Mapping ---
        if (current_data.scanned_char != '\0') {
            char decoded = current_data.scanned_char;
            MovementAction_t action = get_action_for_character(decoded);

            printf("\n===================================================\n");
            printf(">>> BARCODE DECODED: '%c' <<<\n", decoded);

            switch (action) {
                case MOVE_FORWARD:
                    printf(">>> Mapped Action: DRIVE FORWARD STRAIGHT <<<\n");
                    break;
                case MOVE_UTURN:
                    printf(">>> Mapped Action: PERFORM U-TURN <<<\n");
                    break;
                case MOVE_LEFT:
                    printf(">>> Mapped Action: TURN LEFT <<<\n");
                    break;
                case MOVE_RIGHT:
                    printf(">>> Mapped Action: TURN RIGHT <<<\n");
                    break;
                case MOVE_NONE:
                default:
                    printf(">>> Mapped Action: UNMAPPED CHARACTER <<<\n");
                    break;
            }
            printf("===================================================\n\n");
        }

        // Sub-millisecond tick sleep for optimal responsiveness
        sleep_us(500);
    }

    return 0;
}