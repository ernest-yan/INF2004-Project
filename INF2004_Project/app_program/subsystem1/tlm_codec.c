/**
 * @file    tlm_codec.c
 * @brief   Bit packing and command validation for the telemetry link.
 *
 * No hardware or OS calls in this file (unit-tested on the host).
 * Signed values are offset into unsigned ranges before packing
 * (BARR-C 5.3.b: no bitwise operators on signed data).
 */
#include "tlm_codec.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* Bit writer / reader                                                 */
/* ------------------------------------------------------------------ */
typedef struct
{
    uint8_t *p_buf;
    uint32_t   cap_bytes;
    uint32_t   bit_pos;
    bool     b_overflow;
} tlm_bitwriter_t;

typedef struct
{
    const uint8_t *p_buf;
    uint32_t         len_bytes;
    uint32_t         bit_pos;
    bool           b_underflow;
} tlm_bitreader_t;

/* Keep the bit helpers as single shared functions. Without this, -O3 copies
 * them into every encoder and tlm_encode_status() alone grows to ~4 KB. */
#if defined(__GNUC__)
#define TLM_NOINLINE    __attribute__((noinline))
#else
#define TLM_NOINLINE
#endif

#define BITS_PER_BYTE   (8u)
#define PITCH_MIN_DEG   (-64)
#define PITCH_MAX_DEG   (63)
#define PITCH_OFFSET    (64)

static void
bw_init(tlm_bitwriter_t *p_w, uint8_t *p_buf, uint32_t cap)
{
    p_w->p_buf      = p_buf;
    p_w->cap_bytes  = cap;
    p_w->bit_pos    = 0u;
    p_w->b_overflow = false;
    (void) memset(p_buf, 0, cap);
}

/* Write the low 'bits' bits of 'value', most significant bit first. */
static TLM_NOINLINE void
bw_put(tlm_bitwriter_t *p_w, uint32_t value, uint8_t bits)
{
    uint8_t i;

    for (i = bits; i > 0u; i--)
    {
        uint32_t byte_idx = p_w->bit_pos / BITS_PER_BYTE;

        if (byte_idx >= p_w->cap_bytes)
        {
            p_w->b_overflow = true;
            return;
        }

        if (((value >> (i - 1u)) & 1u) != 0u)
        {
            p_w->p_buf[byte_idx] |=
                (uint8_t) (0x80u >> (p_w->bit_pos % BITS_PER_BYTE));
        }
        p_w->bit_pos++;
    }
}

/* Clamp to the largest value the field can hold, then write it. */
static TLM_NOINLINE void
bw_put_sat(tlm_bitwriter_t *p_w, uint32_t value, uint8_t bits)
{
    uint32_t max = ((uint32_t) 1u << bits) - 1u;

    bw_put(p_w, (value > max) ? max : value, bits);
}

static uint32_t
bw_finish(const tlm_bitwriter_t *p_w)
{
    if (p_w->b_overflow)
    {
        return 0u;
    }
    return (p_w->bit_pos + (BITS_PER_BYTE - 1u)) / BITS_PER_BYTE;
}

static TLM_NOINLINE uint32_t
br_get(tlm_bitreader_t *p_r, uint8_t bits)
{
    uint32_t value = 0u;
    uint8_t  i;

    for (i = 0u; i < bits; i++)
    {
        uint32_t byte_idx = p_r->bit_pos / BITS_PER_BYTE;

        if (byte_idx >= p_r->len_bytes)
        {
            p_r->b_underflow = true;
            return 0u;
        }
        value = (value << 1u) |
                ((uint32_t) (p_r->p_buf[byte_idx] >>
                             (7u - (p_r->bit_pos % BITS_PER_BYTE))) & 1u);
        p_r->bit_pos++;
    }
    return value;
}

/* Common 28-bit header: version(4) session(16) seq(8). */
static void
put_header(tlm_bitwriter_t *p_w, uint16_t session, uint8_t seq)
{
    bw_put(p_w, TLM_PROTOCOL_VERSION, 4u);
    bw_put(p_w, session, 16u);
    bw_put(p_w, seq, 8u);
}

/* ------------------------------------------------------------------ */
/* Encoders                                                            */
/* ------------------------------------------------------------------ */
uint32_t
tlm_encode_online(uint16_t session, bool b_online, uint8_t *p_buf, uint32_t cap)
{
    tlm_bitwriter_t w;

    bw_init(&w, p_buf, cap);
    bw_put(&w, TLM_PROTOCOL_VERSION, 4u);
    bw_put(&w, b_online ? 1u : 0u, 1u);
    bw_put(&w, 0u, 3u);                     /* reserved */
    bw_put(&w, session, 16u);
    return bw_finish(&w);
}

uint32_t
tlm_encode_status(const tlm_status_t *p_s, uint8_t *p_buf, uint32_t cap)
{
    tlm_bitwriter_t w;
    int16_t         pitch = p_s->pitch_deg;

    if (pitch < PITCH_MIN_DEG)
    {
        pitch = PITCH_MIN_DEG;
    }
    if (pitch > PITCH_MAX_DEG)
    {
        pitch = PITCH_MAX_DEG;
    }

    bw_init(&w, p_buf, cap);
    put_header(&w, p_s->session, p_s->seq);
    bw_put_sat(&w, p_s->vehicle_state, 3u);
    bw_put(&w, p_s->b_estop_latched ? 1u : 0u, 1u);
    bw_put_sat(&w, p_s->line_state, 2u);
    bw_put_sat(&w, p_s->last_barcode, 3u);
    bw_put_sat(&w, p_s->speed_l_mm_s, 10u);
    bw_put_sat(&w, p_s->speed_r_mm_s, 10u);
    bw_put(&w, p_s->enc_l, 16u);
    bw_put(&w, p_s->enc_r, 16u);
    bw_put_sat(&w, p_s->distance_cm, 12u);
    bw_put(&w, (uint32_t) (pitch + PITCH_OFFSET), 7u);
    bw_put(&w, p_s->hump_max_mm, 8u);
    bw_put_sat(&w, p_s->obstacle_state, 3u);
    bw_put_sat(&w, p_s->obstacle_cm, 9u);
    bw_put(&w, p_s->fault_mask, 8u);
    return bw_finish(&w);
}

uint32_t
tlm_encode_heartbeat(const tlm_heartbeat_t *p_h, uint8_t *p_buf, uint32_t cap)
{
    tlm_bitwriter_t w;

    bw_init(&w, p_buf, cap);
    put_header(&w, p_h->session, p_h->seq);
    bw_put(&w, p_h->uptime_s, 16u);
    bw_put_sat(&w, p_h->rssi_neg_dbm, 7u);
    bw_put_sat(&w, p_h->link_state, 3u);
    bw_put(&w, p_h->cmds_ok, 8u);
    bw_put(&w, p_h->cmds_rejected, 8u);
    bw_put(&w, p_h->queue_full, 8u);
    bw_put(&w, p_h->reconnects, 8u);
    return bw_finish(&w);
}

uint32_t
tlm_encode_ack(uint16_t session, uint8_t seq, uint16_t cmd_seq,
               uint8_t cmd_code, uint8_t result, uint8_t *p_buf, uint32_t cap)
{
    tlm_bitwriter_t w;

    bw_init(&w, p_buf, cap);
    put_header(&w, session, seq);
    bw_put(&w, cmd_seq, 16u);
    bw_put_sat(&w, cmd_code, 4u);
    bw_put_sat(&w, result, 4u);
    return bw_finish(&w);
}

uint32_t
tlm_encode_event(const tlm_event_t *p_e, uint8_t *p_buf, uint32_t cap)
{
    tlm_bitwriter_t w;

    bw_init(&w, p_buf, cap);
    put_header(&w, p_e->session, p_e->seq);
    bw_put_sat(&w, p_e->type, 4u);

    switch (p_e->type)
    {
        case TLM_EVT_STATE_CHANGE:
            bw_put_sat(&w, p_e->a, 3u);
            bw_put_sat(&w, p_e->b, 3u);
            break;

        case TLM_EVT_BARCODE:
        case TLM_EVT_ESTOP:
            bw_put_sat(&w, p_e->a, 3u);
            break;

        case TLM_EVT_HUMP_PEAK:
            bw_put(&w, p_e->a, 8u);
            break;

        case TLM_EVT_OBSTACLE:
            bw_put_sat(&w, p_e->obs_nearest_cm, 9u);
            bw_put_sat(&w, p_e->obs_centre_deg, 8u);
            bw_put(&w, p_e->obs_width_cm, 8u);
            bw_put_sat(&w, p_e->obs_clear_l_cm, 9u);
            bw_put_sat(&w, p_e->obs_clear_r_cm, 9u);
            bw_put_sat(&w, p_e->obs_decision, 3u);
            break;

        default:    /* TLM_EVT_IMPACT and unknown types: header + type only */
            break;
    }
    return bw_finish(&w);
}

/* Tiny JSON writer: no printf, because the T-Kernel build has no C
 * library I/O (newlib's snprintf needs _write/_sbrk, which do not exist). */
typedef struct
{
    char    *p_buf;
    uint32_t cap;
    uint32_t len;
    bool     b_overflow;
} tlm_jsonwriter_t;

static void
jw_char(tlm_jsonwriter_t *p_j, char c)
{
    if ((p_j->len + 1u) >= p_j->cap)        /* keep room for the '\0' */
    {
        p_j->b_overflow = true;
        return;
    }
    p_j->p_buf[p_j->len] = c;
    p_j->len++;
}

static void
jw_str(tlm_jsonwriter_t *p_j, const char *p_s)
{
    while (*p_s != '\0')
    {
        jw_char(p_j, *p_s);
        p_s++;
    }
}

static void
jw_uint(tlm_jsonwriter_t *p_j, uint32_t v)
{
    char     digits[10];
    uint32_t n = 0u;

    do
    {
        digits[n] = (char) ('0' + (v % 10u));
        n++;
        v /= 10u;
    } while (v != 0u);

    while (n > 0u)
    {
        n--;
        jw_char(p_j, digits[n]);
    }
}

/* "name":value, -- one field; signed values arrive as int32_t */
static void
jw_field(tlm_jsonwriter_t *p_j, const char *p_name, int32_t v, bool b_last)
{
    jw_char(p_j, '"');
    jw_str(p_j, p_name);
    jw_str(p_j, "\":");
    if (v < 0)
    {
        jw_char(p_j, '-');
        jw_uint(p_j, (uint32_t) (-(v + 1)) + 1u);   /* safe for INT32_MIN */
    }
    else
    {
        jw_uint(p_j, (uint32_t) v);
    }
    if (!b_last)
    {
        jw_char(p_j, ',');
    }
}

uint32_t
tlm_format_status_json(const tlm_status_t *p_s, char *p_buf, uint32_t cap)
{
    tlm_jsonwriter_t j;

    if (cap == 0u)
    {
        return 0u;
    }
    j.p_buf      = p_buf;
    j.cap        = cap;
    j.len        = 0u;
    j.b_overflow = false;

    jw_char(&j, '{');
    jw_field(&j, "session", (int32_t) p_s->session, false);
    jw_field(&j, "seq", (int32_t) p_s->seq, false);
    jw_field(&j, "state", (int32_t) p_s->vehicle_state, false);
    jw_field(&j, "estop", p_s->b_estop_latched ? 1 : 0, false);
    jw_field(&j, "line", (int32_t) p_s->line_state, false);
    jw_field(&j, "barcode", (int32_t) p_s->last_barcode, false);
    jw_field(&j, "speed_l", (int32_t) p_s->speed_l_mm_s, false);
    jw_field(&j, "speed_r", (int32_t) p_s->speed_r_mm_s, false);
    jw_field(&j, "enc_l", (int32_t) p_s->enc_l, false);
    jw_field(&j, "enc_r", (int32_t) p_s->enc_r, false);
    jw_field(&j, "dist_cm", (int32_t) p_s->distance_cm, false);
    jw_field(&j, "pitch", (int32_t) p_s->pitch_deg, false);
    jw_field(&j, "hump_max_mm", (int32_t) p_s->hump_max_mm, false);
    jw_field(&j, "obs_state", (int32_t) p_s->obstacle_state, false);
    jw_field(&j, "obs_cm", (int32_t) p_s->obstacle_cm, false);
    jw_field(&j, "faults", (int32_t) p_s->fault_mask, true);
    jw_char(&j, '}');

    p_buf[j.len] = '\0';
    return j.b_overflow ? 0u : j.len;
}

/* ------------------------------------------------------------------ */
/* Commands                                                            */
/* ------------------------------------------------------------------ */
uint32_t
tlm_encode_cmd(const tlm_cmd_t *p_cmd, uint8_t *p_buf, uint32_t cap)
{
    tlm_bitwriter_t w;

    bw_init(&w, p_buf, cap);
    bw_put(&w, TLM_PROTOCOL_VERSION, 4u);
    bw_put_sat(&w, p_cmd->code, 4u);
    bw_put(&w, p_cmd->session, 16u);
    bw_put(&w, p_cmd->seq, 16u);
    if (p_cmd->code == (uint8_t) TLM_CMD_MAP)
    {
        uint32_t i;

        for (i = 0u; i < TLM_MAP_LETTERS; i++)
        {
            bw_put(&w, (uint8_t) p_cmd->map[i], 8u);
        }
    }
    return bw_finish(&w);
}

/* Code 39 letters and digits only, and no letter used twice. */
static bool
map_is_valid(const char *p_map)
{
    uint32_t i;
    uint32_t j;

    for (i = 0u; i < TLM_MAP_LETTERS; i++)
    {
        char c = p_map[i];

        if (!(((c >= 'A') && (c <= 'Z')) || ((c >= '0') && (c <= '9'))))
        {
            return false;
        }
        for (j = 0u; j < i; j++)
        {
            if (p_map[j] == c)
            {
                return false;
            }
        }
    }
    return true;
}

/*
 * Checks run in a fixed order; the first failure is reported.
 * p_last_seq is only advanced when every check passes, so a rejected
 * command can never "use up" a sequence number.
 */
tlm_result_t
tlm_cmd_validate(const uint8_t *p_data, uint32_t len, uint16_t current_session,
                 uint16_t *p_last_seq, tlm_cmd_t *p_out)
{
    tlm_bitreader_t r;
    uint32_t        version;
    uint32_t        i;

    p_out->code    = (uint8_t) TLM_CMD_NONE;
    p_out->session = 0u;
    p_out->seq     = 0u;
    for (i = 0u; i < TLM_MAP_LETTERS; i++)
    {
        p_out->map[i] = '\0';
    }

    if ((p_data == NULL) ||
        ((len != TLM_CMD_LEN) && (len != TLM_CMD_MAP_LEN)))
    {
        return TLM_RES_BAD_LEN;
    }

    r.p_buf       = p_data;
    r.len_bytes   = len;
    r.bit_pos     = 0u;
    r.b_underflow = false;

    version        = br_get(&r, 4u);
    p_out->code    = (uint8_t) br_get(&r, 4u);
    p_out->session = (uint16_t) br_get(&r, 16u);
    p_out->seq     = (uint16_t) br_get(&r, 16u);

    if (version != TLM_PROTOCOL_VERSION)
    {
        return TLM_RES_BAD_VERSION;
    }
    /* Session 0 means "not online": nothing is accepted. */
    if ((current_session == 0u) || (p_out->session != current_session))
    {
        return TLM_RES_BAD_SESSION;
    }
    if ((p_out->code < (uint8_t) TLM_CMD_START) ||
        (p_out->code > (uint8_t) TLM_CMD_MAP))
    {
        return TLM_RES_BAD_CMD;
    }
    /* Only MAP carries the 4 letters, so only MAP may be the long form. */
    if ((p_out->code == (uint8_t) TLM_CMD_MAP) != (len == TLM_CMD_MAP_LEN))
    {
        return TLM_RES_BAD_LEN;
    }
    if (p_out->code == (uint8_t) TLM_CMD_MAP)
    {
        for (i = 0u; i < TLM_MAP_LETTERS; i++)
        {
            p_out->map[i] = (char) br_get(&r, 8u);
        }
        if (!map_is_valid(p_out->map))
        {
            return TLM_RES_BAD_CMD;
        }
    }
    if (p_out->seq <= *p_last_seq)
    {
        return TLM_RES_STALE_SEQ;
    }

    *p_last_seq = p_out->seq;
    return TLM_RES_ACCEPTED;
}
