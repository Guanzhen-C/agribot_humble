#include "hitch_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void feedback(hh_controller *s, unsigned at, unsigned position, unsigned limits)
{
    uint8_t data[8] = {position & 255, position >> 8, 0, 0, limits, 0,
                       (uint8_t)(s->sensor_sequence + 1), 0};
    hh_seal(data); assert(hh_sensor(s, data, at));
}
static unsigned command(hh_controller *s, unsigned at, unsigned mode,
                         unsigned target, unsigned sequence, bool enabled)
{
    uint8_t data[8]; hh_make_command(mode, target, 60, sequence, enabled, data);
    return hh_command(s, data, at);
}
int main(void)
{
    assert(hh_crc((const uint8_t *)"123456789", 9) == 0x4b);
    hh_controller s; hh_init(&s); feedback(&s, 100, 4000, 0);
    assert(command(&s, 100, HH_POSITION, 0, 1, true) == HH_UNPRIMED);
    assert(command(&s, 101, HH_HOLD, 0, 2, false) == HH_OK);
    assert(command(&s, 102, HH_POSITION, 9000, 3, true) == HH_OK);
    hh_step(&s, 103, true); assert(s.valve > 0 && s.valve <= 1000);
    assert(command(&s, 104, HH_POSITION, 0, 4, true) == HH_OK);
    hh_step(&s, 105, true); assert(s.valve < 0);
    assert(command(&s, 106, HH_POSITION, 0, 4, true) == HH_REPLAY);
    feedback(&s, 107, 0, 1); hh_step(&s, 108, true); assert(s.valve == 0);
    assert(command(&s, 109, HH_RAISE, 0, 5, true) == HH_OK);
    hh_step(&s, 110, true); assert(s.valve > 0);
    feedback(&s, 111, 10000, 2); hh_step(&s, 112, true); assert(s.valve == 0);
    feedback(&s, 113, 6000, 0);
    assert(command(&s, 114, HH_POSITION, 9000, 6, true) == HH_OK);
    hh_step(&s, 115, true); assert(s.valve > 0);
    hh_step(&s, 314, true); assert(s.valve == 0 && !s.primed);
    feedback(&s, 315, 6000, 0);
    assert(command(&s, 316, HH_POSITION, 9000, 7, true) == HH_UNPRIMED);
    assert(command(&s, 317, HH_HOLD, 0, 8, false) == HH_OK);
    assert(command(&s, 318, HH_POSITION, 9000, 9, true) == HH_OK);
    hh_step(&s, 319, false); assert(s.valve == 0 && !s.primed);
    assert(command(&s, 320, HH_HOLD, 0, 10, false) == HH_OK);
    assert(command(&s, 321, HH_POSITION, 9000, 11, true) == HH_OK);
    feedback(&s, 822, 5000, 0); hh_step(&s, 823, true);
    assert(s.valve == 0 && !s.primed); /* command timeout, despite fresh feedback */
    uint8_t bad[8]; hh_make_command(HH_POSITION, 10001, 50, 12, true, bad);
    assert(hh_command(&s, bad, 824) == HH_RANGE);
    hh_make_command(HH_HOLD, 0, 50, 12, false, bad); bad[2] ^= 1;
    assert(hh_command(&s, bad, 824) == HH_FORMAT);
    hh_init(&s); feedback(&s, 0xfffffff0U, 5000, 0);
    assert(command(&s, 0xfffffff1U, HH_HOLD, 0, 254, false) == HH_OK);
    assert(command(&s, 0xfffffff2U, HH_POSITION, 7000, 255, true) == HH_OK);
    assert(command(&s, 5, HH_POSITION, 7000, 0, true) == HH_OK);
    hh_step(&s, 6, true); assert(s.valve > 0); /* clock and sequence rollover */
    uint8_t out[8]; hh_output(&s, out); assert(hh_valid(out) && out[5] == HH_VERSION);
    puts("hitch controller: CRC, positions, hold, limits, replay, stale, restart, rollovers PASS");
    return 0;
}
