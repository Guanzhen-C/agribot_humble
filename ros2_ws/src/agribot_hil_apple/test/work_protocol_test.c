#include "work_protocol.h"
#include <assert.h>
#include <string.h>

static void feedback(hil_work_gateway_t *s, uint32_t now, uint8_t counter)
{
    uint8_t b[8] = {1}; b[6] = counter; b[7] = hil_work_xor(b, 7);
    assert(hil_work_feedback(s, 0x532, b, now));
    for (unsigned i = 0; i < 2; ++i) {
        memset(b, 0, 8); b[0] = i; b[6] = counter; b[7] = hil_work_xor(b, 7);
        assert(hil_work_feedback(s, 0x5b0, b, now));
    }
    for (unsigned i = 0; i < 3; ++i) {
        memset(b, 0, 8); b[6] = counter; b[7] = hil_work_xor(b, 7);
        assert(hil_work_feedback(s, 0x5b2 + i, b, now));
    }
}

int main(void)
{
    hil_work_gateway_t s; uint8_t b[8], out[8];
    hil_work_init(&s);
    hil_work_command(1, 1, 1, true, b);
    assert(hil_work_accept(&s, b, 10) == HIL_WORK_UNPRIMED);
    feedback(&s, 10, 0);
    for (unsigned i = 1; i <= 3; ++i) {
        hil_work_command(i, 0, 1, false, b);
        assert(hil_work_accept(&s, b, 10) == HIL_WORK_OK);
        hil_work_command(i, i == 3 ? 4321 : i + 2, 2, true, b);
        assert(hil_work_accept(&s, b, 20) == HIL_WORK_OK);
    }
    hil_work_encode(&s, 20, out);
    assert(out[0] == 0x43 && out[1] == 0xe1 && out[2] == 0x10);
    assert(out[6] == 0 && out[7] == 0xb2);
    assert(hil_work_accept(&s, b, 30) == HIL_WORK_REPLAY);
    hil_work_command(1, 9, 3, true, b);
    assert(hil_work_accept(&s, b, 30) == HIL_WORK_RANGE);
    /* Renew only fan: pump and shredder must still expire independently. */
    hil_work_command(1, 3, 3, true, b);
    assert(hil_work_accept(&s, b, 250) == HIL_WORK_OK);
    hil_work_encode(&s, 321, out);
    assert(out[0] == 3 && out[1] == 0 && out[2] == 0);
    hil_work_command(2, 4, 3, true, b);
    assert(hil_work_accept(&s, b, 330) == HIL_WORK_UNPRIMED);
    hil_work_stop(&s); hil_work_encode(&s, 331, out);
    assert(out[0] == 0 && out[1] == 0 && out[2] == 0);
    /* Replayed feedback never renews freshness. */
    memset(b, 0, 8);
    assert(!hil_work_feedback(&s, 0x5b2, b, 1000));
    assert(!hil_work_ready(&s, 0, 1510));
    /* Unsigned timer rollover preserves the 300 ms command lease. */
    hil_work_init(&s); feedback(&s, UINT32_MAX - 50, 0);
    hil_work_command(3, 0, 254, false, b);
    assert(hil_work_accept(&s, b, UINT32_MAX - 50) == HIL_WORK_OK);
    hil_work_command(3, 10000, 255, true, b);
    assert(hil_work_accept(&s, b, UINT32_MAX - 40) == HIL_WORK_OK);
    hil_work_command(3, 10000, 0, true, b);
    assert(hil_work_accept(&s, b, 5) == HIL_WORK_OK);
    hil_work_encode(&s, 6, out); assert(out[1] == 0x10 && out[2] == 0x27);
    hil_work_encode(&s, 305, out); assert(out[1] == 0 && out[2] == 0);
    /* A restarted controller may resynchronize only with an expired neutral. */
    hil_work_command(3, 0, 200, false, b);
    assert(hil_work_accept(&s, b, 306) == HIL_WORK_OK);
    hil_work_command(3, 0, 1, false, b);
    assert(hil_work_accept(&s, b, 607) == HIL_WORK_OK);
    /* Minimum liquid level/sensor abnormality prevents pump activation. */
    memset(b, 0, 8); b[0] = 0x80; b[6] = 1; b[7] = hil_work_xor(b, 7);
    assert(hil_work_feedback(&s, 0x5b3, b, 608));
    assert(!hil_work_ready(&s, 1, 608));
    return 0;
}
