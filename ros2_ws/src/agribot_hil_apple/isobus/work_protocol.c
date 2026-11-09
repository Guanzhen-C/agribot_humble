#include "work_protocol.h"
#include <string.h>

uint8_t hil_work_xor(const uint8_t *bytes, unsigned length)
{
    uint8_t result = 0;
    for (unsigned i = 0; i < length; ++i) { result ^= bytes[i]; }
    return result;
}

void hil_work_init(hil_work_gateway_t *state) { memset(state, 0, sizeof(*state)); }

void hil_work_stop(hil_work_gateway_t *state)
{
    for (unsigned i = 0; i < 3; ++i) {
        state->channels[i].enabled = state->channels[i].primed = false;
        state->channels[i].value = 0;
    }
}

static bool fresh(bool seen, uint32_t at, uint32_t now)
{
    return seen && (uint32_t)(now - at) < HIL_WORK_FEEDBACK_MS;
}

bool hil_work_ready(const hil_work_gateway_t *state, unsigned channel, uint32_t now)
{
    if (channel > 2) { return false; }
    unsigned common = channel == 2 ? 1U : 0U;
    return fresh(state->vehicle_seen, state->vehicle_at, now) && state->vehicle_safe &&
        fresh(state->common_seen[common], state->common_at[common], now) &&
        !state->common_fault[common] && !state->channel_fault[channel] &&
        fresh(state->feedback_seen[channel], state->feedback_at[channel], now);
}

static bool next_counter(bool seen, uint8_t previous, uint8_t current)
{
    uint8_t advance = (uint8_t)((current - previous) & 15U);
    return !seen || (advance > 0 && advance <= 8);
}

bool hil_work_feedback(hil_work_gateway_t *state, uint16_t id,
                       const uint8_t bytes[8], uint32_t now)
{
    if (!bytes || hil_work_xor(bytes, 7) != bytes[7]) { return false; }
    uint8_t counter = bytes[6] & 15U;
    if (id == 0x532) {
        if (!next_counter(state->vehicle_seen && fresh(true, state->vehicle_at, now),
                          state->vehicle_counter, counter)) { return false; }
        state->vehicle_counter = counter;
        state->vehicle_seen = true;
        state->vehicle_at = now;
        state->vehicle_safe = (bytes[0] & 7U) == 1U && !(bytes[4] & 15U);
    } else if (id == 0x5b0 && bytes[0] < 2) {
        unsigned common = bytes[0];
        if (!next_counter(state->common_seen[common] && fresh(true, state->common_at[common], now),
                          state->common_counter[common], counter)) { return false; }
        state->common_seen[common] = true;
        state->common_counter[common] = counter;
        state->common_at[common] = now;
        state->common_fault[common] = common == 0 ? (bytes[1] & 3U) != 0 : (bytes[2] & 7U) != 0;
    } else if (id >= 0x5b2 && id <= 0x5b4) {
        unsigned channel = id - 0x5b2;
        if (!next_counter(state->feedback_seen[channel] && fresh(true, state->feedback_at[channel], now),
                          state->feedback_counter[channel], counter)) { return false; }
        state->feedback_seen[channel] = true;
        state->feedback_counter[channel] = counter;
        state->feedback_at[channel] = now;
        state->channel_fault[channel] = channel == 2 ? bytes[0] != 0 : (bytes[0] & 15U) > 8;
        if (channel == 1 && (bytes[0] >> 6) >= 2) { state->channel_fault[channel] = true; }
    } else { return false; }
    return true;
}

uint8_t hil_work_accept(hil_work_gateway_t *state, const uint8_t bytes[8], uint32_t now)
{
    uint8_t result = HIL_WORK_FORMAT;
    if (!bytes || bytes[0] != HIL_WORK_VERSION || hil_work_xor(bytes, 7) != bytes[7]) {
        ++state->rejected; return result;
    }
    if (bytes[1] == HIL_WORK_STOP && bytes[3] == 0 && bytes[4] == 0 &&
        bytes[5] == 0 && bytes[6] == 0) {
        hil_work_stop(state); ++state->accepted; return HIL_WORK_OK;
    }
    if (bytes[1] != HIL_WORK_SET || bytes[3] < 1 || bytes[3] > 3 || bytes[6] > 1) {
        ++state->rejected; return result;
    }
    unsigned index = bytes[3] - 1U;
    uint16_t value = (uint16_t)(bytes[4] | (uint16_t)bytes[5] << 8);
    hil_work_channel_t *channel = &state->channels[index];
    if (value > (index == 2 ? 10000 : 8) || (!bytes[6] && value)) {
        result = HIL_WORK_RANGE;
    } else if (channel->has_sequence && (value ||
                (uint32_t)(now - channel->received_at) < HIL_WORK_LEASE_MS) &&
               ((uint8_t)(bytes[2] - channel->sequence) == 0 ||
                (uint8_t)(bytes[2] - channel->sequence) > 127)) {
        result = HIL_WORK_REPLAY;
    } else if (value && (!channel->primed ||
                        (uint32_t)(now - channel->received_at) >= HIL_WORK_LEASE_MS)) {
        channel->primed = channel->enabled = false;
        channel->value = 0;
        result = HIL_WORK_UNPRIMED;
    } else if (value && !hil_work_ready(state, index, now)) {
        channel->primed = channel->enabled = false;
        channel->value = 0;
        result = HIL_WORK_NOT_READY;
    } else {
        channel->sequence = bytes[2]; channel->has_sequence = true;
        channel->received_at = now; channel->enabled = bytes[6] != 0;
        channel->value = value; channel->primed = true;
        ++state->accepted; return HIL_WORK_OK;
    }
    ++state->rejected;
    return result;
}

void hil_work_encode(hil_work_gateway_t *state, uint32_t now, uint8_t bytes[8])
{
    uint16_t values[3] = {0};
    memset(bytes, 0, 8);
    for (unsigned i = 0; i < 3; ++i) {
        hil_work_channel_t *channel = &state->channels[i];
        if (!channel->has_sequence || (uint32_t)(now - channel->received_at) >= HIL_WORK_LEASE_MS ||
            !hil_work_ready(state, i, now)) {
            channel->primed = channel->enabled = false;
            channel->value = 0;
        }
        if (channel->enabled) { values[i] = channel->value; }
    }
    bytes[0] = (uint8_t)(values[0] | values[1] << 4);
    bytes[1] = values[2] & 255U; bytes[2] = values[2] >> 8;
    bytes[6] = state->native_counter++ & 15U;
    bytes[7] = hil_work_xor(bytes, 7);
}

void hil_work_command(unsigned device, uint16_t value, uint8_t sequence,
                      bool enabled, uint8_t bytes[8])
{
    memset(bytes, 0, 8);
    bytes[0] = HIL_WORK_VERSION; bytes[1] = HIL_WORK_SET;
    bytes[2] = sequence; bytes[3] = (uint8_t)device;
    bytes[4] = value & 255U; bytes[5] = value >> 8; bytes[6] = enabled ? 1 : 0;
    bytes[7] = hil_work_xor(bytes, 7);
}
