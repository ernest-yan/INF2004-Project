/* Prints BARCODE event frames as hex, one per line:
 * "<action> <code> <scan_result> <elements> <hex>".
 * test_barcode_event.py decodes them with dashboard.py's decoder. */
#include "tlm_codec.h"
#include <stdio.h>

static void emit(unsigned action, unsigned code, unsigned result, unsigned elems)
{
    tlm_event_t e = {0};
    uint8_t buf[TLM_EVENT_MAX_LEN];
    e.session = 4242U; e.seq = 9U; e.type = TLM_EVT_BARCODE;
    e.a = (uint8_t)action; e.b = (uint8_t)code;
    e.scan_result = (uint8_t)result; e.scan_elems = (uint8_t)elems;
    const uint32_t n = tlm_encode_event(&e, buf, sizeof(buf));
    printf("%u %u %u %u ", action, code, result, elems);
    for (uint32_t i = 0U; i < n; i++) { printf("%02x", buf[i]); }
    printf("\n");
}

int main(void)
{
    for (unsigned code = 1U; code <= 36U; code++) {
        emit(1U + (code % 5U), code, 0U, 29U + (code % 31U));  /* decoded, 29..59 */
    }
    emit(0U, 0U, 1U, 4U);   /* seen, not decoded: too short (4 bars+spaces) */
    emit(0U, 0U, 1U, 28U);  /* too short, one under a full barcode */
    emit(0U, 0U, 2U, 31U);  /* no '*' start/stop frame */
    emit(0U, 0U, 3U, 59U);  /* framed, letter unreadable; most elements */
    emit(5U, 5U, 0U, 29U);  /* 'E' not in the map: INVALID, letter still sent */
    return 0;
}
