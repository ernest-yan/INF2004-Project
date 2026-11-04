/**
 * Host unit tests for tlm_codec.c (runs on your laptop, no Pico needed).
 *
 * From the repo root (WSL / Linux / Mac):
 *   gcc -std=c11 -Wall -Wextra -Iapp_program/subsystem1 \
 *       app_program/subsystem1/tlm_codec.c testing/subsystem1/host_test/test_tlm_codec.c \
 *       -o test_codec && ./test_codec
 */
#include "tlm_codec.h"

#include <stdio.h>
#include <string.h>

static int g_failures = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            printf("FAIL line %d: %s\n", __LINE__, #cond);               \
            g_failures++;                                                \
        }                                                                \
    } while (0)

static uint32_t
make_cmd(uint8_t code, uint16_t session, uint16_t seq, uint8_t *p_buf)
{
    tlm_cmd_t c = { code, session, seq, { 0 } };
    return tlm_encode_cmd(&c, p_buf, TLM_CMD_LEN);
}

static void
test_cmd_validation(void)
{
    uint8_t   buf[TLM_CMD_LEN];
    uint16_t  last_seq = 0u;
    tlm_cmd_t out;
    const uint16_t session = 0x4A3Bu;

    /* Known byte layout: ver=1, cmd=START(1), session=0x4A3B, seq=1 */
    CHECK(make_cmd(TLM_CMD_START, session, 1u, buf) == TLM_CMD_LEN);
    CHECK(buf[0] == 0x11u && buf[1] == 0x4Au && buf[2] == 0x3Bu &&
          buf[3] == 0x00u && buf[4] == 0x01u);

    /* 1. valid command accepted, seq advances */
    CHECK(tlm_cmd_validate(buf, TLM_CMD_LEN, session, &last_seq, &out)
          == TLM_RES_ACCEPTED);
    CHECK(out.code == TLM_CMD_START && last_seq == 1u);

    /* 2. same message again (replay / QoS 1 duplicate) rejected */
    CHECK(tlm_cmd_validate(buf, TLM_CMD_LEN, session, &last_seq, &out)
          == TLM_RES_STALE_SEQ);

    /* 3. command from an old session (stale or retained) rejected */
    make_cmd(TLM_CMD_START, 0x1111u, 2u, buf);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_LEN, session, &last_seq, &out)
          == TLM_RES_BAD_SESSION);
    CHECK(last_seq == 1u);      /* rejected command did not move seq */

    /* 4. not online (session 0) rejects everything */
    make_cmd(TLM_CMD_STOP, 0u, 9u, buf);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_LEN, 0u, &last_seq, &out)
          == TLM_RES_BAD_SESSION);

    /* 5. wrong length */
    make_cmd(TLM_CMD_STOP, session, 2u, buf);
    CHECK(tlm_cmd_validate(buf, 4u, session, &last_seq, &out)
          == TLM_RES_BAD_LEN);
    CHECK(tlm_cmd_validate(NULL, 0u, session, &last_seq, &out)
          == TLM_RES_BAD_LEN);

    /* 6. wrong version */
    buf[0] = (uint8_t) ((2u << 4) | TLM_CMD_STOP);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_LEN, session, &last_seq, &out)
          == TLM_RES_BAD_VERSION);

    /* 7. unknown command code */
    make_cmd(9u, session, 3u, buf);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_LEN, session, &last_seq, &out)
          == TLM_RES_BAD_CMD);

    /* 8. E-STOP with a newer seq accepted */
    make_cmd(TLM_CMD_ESTOP, session, 7u, buf);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_LEN, session, &last_seq, &out)
          == TLM_RES_ACCEPTED);
    CHECK(out.code == TLM_CMD_ESTOP && last_seq == 7u);
}

static uint32_t
make_map(const char *p_letters, uint16_t session, uint16_t seq, uint8_t *p_buf)
{
    tlm_cmd_t c = { TLM_CMD_MAP, session, seq, { 0 } };

    memcpy(c.map, p_letters, TLM_MAP_LETTERS);
    return tlm_encode_cmd(&c, p_buf, TLM_CMD_MAP_LEN);
}

static void
test_reset_and_map(void)
{
    uint8_t   buf[TLM_CMD_MAP_LEN];
    uint16_t  last_seq = 0u;
    tlm_cmd_t out;
    const uint16_t session = 0x4A3Bu;

    /* 1. RESET is a normal 5-byte command */
    make_cmd(TLM_CMD_RESET, session, 1u, buf);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_LEN, session, &last_seq, &out)
          == TLM_RES_ACCEPTED);
    CHECK(out.code == TLM_CMD_RESET);

    /* 2. MAP: 9 bytes, letters come back in order */
    CHECK(make_map("FULR", session, 2u, buf) == TLM_CMD_MAP_LEN);
    CHECK(buf[5] == 'F' && buf[8] == 'R');
    CHECK(tlm_cmd_validate(buf, TLM_CMD_MAP_LEN, session, &last_seq, &out)
          == TLM_RES_ACCEPTED);
    CHECK(out.code == TLM_CMD_MAP && memcmp(out.map, "FULR", 4) == 0);
    CHECK(last_seq == 2u);

    /* 3. digits allowed (Code 39) */
    make_map("1234", session, 3u, buf);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_MAP_LEN, session, &last_seq, &out)
          == TLM_RES_ACCEPTED);

    /* 4. same letter twice, lowercase, or a symbol: BAD_CMD, seq not used */
    make_map("FFLR", session, 4u, buf);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_MAP_LEN, session, &last_seq, &out)
          == TLM_RES_BAD_CMD);
    make_map("fulr", session, 4u, buf);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_MAP_LEN, session, &last_seq, &out)
          == TLM_RES_BAD_CMD);
    make_map("FU-R", session, 4u, buf);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_MAP_LEN, session, &last_seq, &out)
          == TLM_RES_BAD_CMD);
    CHECK(last_seq == 3u);

    /* 5. MAP cut to 5 bytes, or START padded to 9: BAD_LEN */
    make_map("FULR", session, 4u, buf);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_LEN, session, &last_seq, &out)
          == TLM_RES_BAD_LEN);
    make_cmd(TLM_CMD_START, session, 4u, buf);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_MAP_LEN, session, &last_seq, &out)
          == TLM_RES_BAD_LEN);

    /* 6. MAP from an old session is rejected like any command */
    make_map("FULR", 0x1111u, 4u, buf);
    CHECK(tlm_cmd_validate(buf, TLM_CMD_MAP_LEN, session, &last_seq, &out)
          == TLM_RES_BAD_SESSION);
    CHECK(last_seq == 3u);
}

static void
test_status_frame(void)
{
    tlm_status_t s;
    uint8_t      buf[32];
    char         json[300];
    uint32_t       n;
    uint32_t       j;

    memset(&s, 0, sizeof(s));
    s.session = 0x4A3Bu;
    s.seq = 200u;
    s.vehicle_state = 2u;
    s.b_estop_latched = true;
    s.line_state = 1u;
    s.last_barcode = 4u;
    s.speed_l_mm_s = 350u;
    s.speed_r_mm_s = 5000u;        /* too big: must saturate to 1023 */
    s.enc_l = 12345u;
    s.enc_r = 65535u;
    s.distance_cm = 1234u;
    s.pitch_deg = -12;
    s.hump_max_mm = 42u;
    s.obstacle_state = 5u;
    s.obstacle_cm = 87u;
    s.fault_mask = 0xA5u;

    n = tlm_encode_status(&s, buf, sizeof(buf));
    CHECK(n == TLM_STATUS_LEN);
    CHECK(tlm_encode_status(&s, buf, 10u) == 0u);   /* too small buffer */

    j = tlm_format_status_json(&s, json, sizeof(json));
    CHECK(j > 0u);
    printf("status: binary %u bytes vs JSON %u bytes (%.0f%% smaller)\n",
           (unsigned) TLM_STATUS_LEN, (unsigned) j,
           100.0 * (1.0 - ((double) TLM_STATUS_LEN / (double) j)));

    /* Print the frame so tools/dashboard.py can be checked against it. */
    printf("status hex: ");
    for (j = 0u; j < n; j++)
    {
        printf("%02X", buf[j]);
    }
    printf("\n");
}

static void
test_other_frames(void)
{
    uint8_t         buf[32];
    tlm_heartbeat_t h;
    tlm_event_t     e;

    CHECK(tlm_encode_online(0x4A3Bu, true, buf, sizeof(buf)) == TLM_ONLINE_LEN);
    CHECK(buf[0] == 0x18u && buf[1] == 0x4Au && buf[2] == 0x3Bu);

    memset(&h, 0, sizeof(h));
    CHECK(tlm_encode_heartbeat(&h, buf, sizeof(buf)) == TLM_HEARTBEAT_LEN);
    CHECK(tlm_encode_ack(1u, 2u, 3u, TLM_CMD_START, TLM_RES_ACCEPTED,
                         buf, sizeof(buf)) == TLM_ACK_LEN);

    memset(&e, 0, sizeof(e));
    e.type = TLM_EVT_OBSTACLE;
    CHECK(tlm_encode_event(&e, buf, sizeof(buf)) == TLM_EVENT_MAX_LEN);
    e.type = TLM_EVT_IMPACT;
    CHECK(tlm_encode_event(&e, buf, sizeof(buf)) == 4u);
}

int
main(void)
{
    test_cmd_validation();
    test_reset_and_map();
    test_status_frame();
    test_other_frames();

    if (g_failures == 0)
    {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("%d TEST(S) FAILED\n", g_failures);
    return 1;
}
