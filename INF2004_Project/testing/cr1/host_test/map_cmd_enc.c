/* Prints MAP command frames from the C codec as "<letters> <hex>", and the
 * validator's verdict on frames given on stdin. test_map_cmd.py compares
 * them with dashboard.py's encode_cmd(). */
#include "tlm_codec.h"
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    (void)argv;
    static const char *maps[] = { "CDAB", "FULR", "0129", "ZYXW", "A9B8" };
    uint8_t buf[TLM_CMD_MAP_LEN];
    if (argc > 1)       /* validate hex frames from stdin: print result codes */
    {
        char hex[64];
        uint16_t last = 0u;
        while (scanf("%63s", hex) == 1)
        {
            uint32_t n = (uint32_t)strlen(hex) / 2u;
            tlm_cmd_t out;
            for (uint32_t i = 0u; i < n; i++) { sscanf(&hex[2u * i], "%2hhx", &buf[i]); }
            tlm_result_t r = tlm_cmd_validate(buf, n, 0x4A3Bu, &last, &out);
            printf("%d %.4s\n", (int)r, (r == TLM_RES_ACCEPTED) ? out.map : "----");
        }
        return 0;
    }
    for (unsigned m = 0U; m < 5U; m++)
    {
        tlm_cmd_t c = { TLM_CMD_MAP, 0x4A3Bu, (uint16_t)(10U + m), { 0 } };
        memcpy(c.map, maps[m], TLM_MAP_LETTERS);
        const uint32_t n = tlm_encode_cmd(&c, buf, sizeof(buf));
        printf("%s %u ", maps[m], 10U + m);
        for (uint32_t i = 0U; i < n; i++) { printf("%02x", buf[i]); }
        printf("\n");
    }
    return 0;
}
