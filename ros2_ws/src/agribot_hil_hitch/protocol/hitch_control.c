#include "hitch_control.h"
#include <string.h>

uint8_t hh_crc(const uint8_t *data, unsigned length)
{
    uint8_t crc = 0xff;
    for (unsigned i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x1d : crc << 1);
        }
    }
    return crc ^ 0xff;
}
bool hh_valid(const uint8_t data[8]) { return data && hh_crc(data, 7) == data[7]; }
void hh_seal(uint8_t data[8]) { data[7] = hh_crc(data, 7); }
bool hh_next(uint8_t previous, uint8_t current)
{
    uint8_t difference = (uint8_t)(current - previous);
    return difference > 0 && difference <= 127;
}
static uint16_t u16(const uint8_t *p) { return p[0] | (uint16_t)p[1] << 8; }
static bool recent(bool seen, uint32_t at, uint32_t now, unsigned timeout)
{ return seen && (uint32_t)(now - at) < timeout; }
static int clamp(int value, int lower, int upper)
{ return value < lower ? lower : value > upper ? upper : value; }

void hh_init(hh_controller *state) { memset(state, 0, sizeof(*state)); }
void hh_stop(hh_controller *state)
{
    state->enabled = state->primed = false;
    state->mode = HH_HOLD;
    state->valve = 0;
}
bool hh_sensor(hh_controller *state, const uint8_t data[8], uint32_t now)
{
    if (!hh_valid(data) || u16(data) > 10000 || (data[4] & ~3U) || data[4] == 3) {
        return false;
    }
    if (recent(state->have_sensor, state->sensor_at, now, HH_SENSOR_TIMEOUT) &&
        !hh_next(state->sensor_sequence, data[6])) { return false; }
    state->position = u16(data);
    state->velocity = (int16_t)u16(data + 2);
    state->limits = data[4]; state->sensor_fault = data[5];
    state->sensor_sequence = data[6];
    state->sensor_at = now; state->have_sensor = true;
    return true;
}
uint8_t hh_command(hh_controller *state, const uint8_t data[8], uint32_t now)
{
    if (!hh_valid(data) || data[0] != HH_VERSION || data[1] > HH_LOWER || data[6] > 1) {
        return HH_FORMAT;
    }
    uint16_t target = u16(data + 2);
    if (target > 10000 || data[4] > 100 ||
        (data[1] != HH_HOLD && (!data[4] || !data[6]))) { return HH_RANGE; }
    /* A disabled HOLD is the only restart/authorization priming operation.
       POSITION 0 is a real lowering target and must NEVER prime the channel. */
    bool neutral = data[1] == HH_HOLD && data[6] == 0;
    bool fresh = recent(state->have_command, state->command_at, now, HH_COMMAND_TIMEOUT);
    if (state->have_command && (fresh || !neutral) && !hh_next(state->sequence, data[5])) {
        return HH_REPLAY;
    }
    if (!neutral && (!fresh || !state->primed)) { hh_stop(state); return HH_UNPRIMED; }
    if (!neutral && (!recent(state->have_sensor, state->sensor_at, now, HH_SENSOR_TIMEOUT) ||
        state->sensor_fault)) { hh_stop(state); return HH_NOT_READY; }
    state->mode = data[1]; state->target = target; state->speed = data[4];
    state->enabled = data[6] != 0; state->primed = true;
    state->sequence = data[5]; state->command_at = now; state->have_command = true;
    if (state->mode == HH_HOLD) { state->valve = 0; }
    return HH_OK;
}
void hh_step(hh_controller *state, uint32_t now, bool network_ready)
{
    state->fault = 0;
    bool sensor_fresh = recent(state->have_sensor, state->sensor_at, now, HH_SENSOR_TIMEOUT);
    bool command_fresh = recent(state->have_command, state->command_at, now, HH_COMMAND_TIMEOUT);
    if (!sensor_fresh) { state->fault |= HH_FAULT_STALE; }
    if (state->sensor_fault) { state->fault |= HH_FAULT_SENSOR; }
    if (!network_ready || !sensor_fresh || !command_fresh || state->sensor_fault) {
        hh_stop(state); return;
    }
    if (!state->enabled || state->mode == HH_HOLD) { state->valve = 0; return; }
    int error = (int)state->target - state->position;
    int max_velocity = state->speed * 30;
    int desired_velocity = state->mode == HH_RAISE ? max_velocity :
                           state->mode == HH_LOWER ? -max_velocity :
                           clamp(error * 3, -max_velocity, max_velocity);
    if (state->mode == HH_POSITION && error >= -40 && error <= 40) {
        desired_velocity = 0;
    }
    int output = clamp((desired_velocity * 10 - state->velocity * 2) / 30, -1000, 1000);
    if (!desired_velocity) { output = 0; }
    if (((state->limits & 1) && output < 0) || ((state->limits & 2) && output > 0)) {
        output = 0;
    }
    state->valve = (int16_t)output;
}
void hh_output(hh_controller *state, uint8_t data[8])
{
    memset(data, 0, 8);
    uint16_t valve = (uint16_t)state->valve;
    data[0] = valve & 255; data[1] = valve >> 8;
    data[2] = state->enabled && state->primed && !state->fault;
    data[3] = state->mode; data[4] = state->fault; data[5] = HH_VERSION;
    data[6] = state->native_sequence++;
    hh_seal(data);
}
void hh_make_command(uint8_t mode, uint16_t target, uint8_t speed,
                     uint8_t sequence, bool enabled, uint8_t data[8])
{
    data[0] = HH_VERSION; data[1] = mode;
    data[2] = target & 255; data[3] = target >> 8;
    data[4] = speed; data[5] = sequence; data[6] = enabled;
    hh_seal(data);
}
