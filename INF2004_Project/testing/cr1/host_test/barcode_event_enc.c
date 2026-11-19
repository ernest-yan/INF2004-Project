/* Prints BARCODE event frames as hex, one per line: "<action> <code> <hex>".
 * test_barcode_event.py decodes them with dashboard.py's decoder. */
#include "tlm_codec.h"
#include <stdio.h>

static void emit(unsigned action, unsigned code)
{
    tlm_event_t e = {0};
    uint8_t buf[TLM_EVENT_MAX_LEN];
    e.session = 4242U; e.seq = 9U; e.type = TLM_EVT_BARCODE;
    e.a = (uint8_t)action; e.b = (uint8_t)code;
    const uint32_t n = tlm_encode_event(&e, buf, sizeof(buf));
    printf("%u %u ", action, code);
    for (uint32_t i = 0U; i < n; i++) { printf("%02x", buf[i]); }
    printf("\n");
}

int main(void)
{
    for (unsigned code = 1U; code <= 36U; code++) { emit(1U + (code % 5U), code); }
    emit(0U, 0U);       /* seen, not decoded */
    emit(5U, 5U);       /* 'E' not in the map: INVALID, letter still sent */
    return 0;
}
