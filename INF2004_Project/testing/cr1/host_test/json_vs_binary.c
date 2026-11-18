/**
 * CR1 message-size evidence: the same telemetry STATUS frame as text (JSON,
 * tlm_format_status_json) and as the packed binary frame (tlm_encode_status).
 *   gcc -std=c11 -Iapp_program/subsystem1 app_program/subsystem1/tlm_codec.c \
 *       testing/cr1/host_test/json_vs_binary.c -o jvb && ./jvb
 */
#include "tlm_codec.h"
#include <stdio.h>

static void show(const char *name, const tlm_status_t *s)
{
    char json[512];
    uint8_t bin[TLM_STATUS_LEN];
    const uint32_t nj = tlm_format_status_json(s, json, sizeof(json));
    const uint32_t nb = tlm_encode_status(s, bin, sizeof(bin));
    printf("%-9s JSON %3u B, binary %2u B (%.1f%% of the text size)\n  %s\n",
           name, (unsigned)nj, (unsigned)nb, 100.0 * nb / nj, json);
}

int main(void)
{
    tlm_status_t s = {0};
    s.session = 4242U; s.seq = 17U; s.vehicle_state = 2U; s.line_state = 1U;
    s.last_barcode = 3U; s.speed_l_mm_s = 70U; s.speed_r_mm_s = 68U;
    s.enc_l = 12345U; s.enc_r = 12301U; s.distance_cm = 287U; s.pitch_deg = 1;
    s.hump_max_mm = 10U; s.obstacle_cm = 85U; s.yaw_dps = -3; s.motion = 2U;
    s.up_speed_mm_s = 0U; s.tilt_deg = 2U;
    show("typical", &s);
    s.session = 65535U; s.seq = 255U; s.vehicle_state = 6U; s.b_estop_latched = true;
    s.line_state = 3U; s.last_barcode = 5U; s.speed_l_mm_s = 1023U;
    s.speed_r_mm_s = 1023U; s.enc_l = 65535U; s.enc_r = 65535U;
    s.distance_cm = 4095U; s.pitch_deg = -64; s.hump_max_mm = 255U;
    s.obstacle_state = 7U; s.obstacle_cm = 511U; s.fault_mask = 0x3CU;
    s.yaw_dps = -256; s.motion = 6U; s.up_speed_mm_s = 1020U; s.tilt_deg = 127U;
    s.s5_fault = 6U;
    show("largest", &s);
    return 0;
}
