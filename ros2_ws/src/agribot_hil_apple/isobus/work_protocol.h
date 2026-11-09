#ifndef AGRIBOT_HIL_WORK_PROTOCOL_H
#define AGRIBOT_HIL_WORK_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { HIL_WORK_VERSION = 1, HIL_WORK_SET = 1, HIL_WORK_STOP = 2 };
enum { HIL_WORK_OK, HIL_WORK_FORMAT, HIL_WORK_RANGE, HIL_WORK_REPLAY,
       HIL_WORK_UNPRIMED, HIL_WORK_NOT_READY };
enum { HIL_WORK_LEASE_MS = 300, HIL_WORK_FEEDBACK_MS = 1500 };

typedef struct {
    uint16_t value;
    uint32_t received_at;
    uint8_t sequence;
    bool has_sequence, primed, enabled;
} hil_work_channel_t;

typedef struct {
    hil_work_channel_t channels[3];
    uint32_t feedback_at[3], common_at[2], vehicle_at;
    uint8_t feedback_counter[3], common_counter[2], vehicle_counter;
    bool feedback_seen[3], common_seen[2], vehicle_seen;
    bool common_fault[2], channel_fault[3], vehicle_safe;
    uint32_t accepted, rejected;
    uint8_t native_counter;
} hil_work_gateway_t;

uint8_t hil_work_xor(const uint8_t *bytes, unsigned length);
void hil_work_init(hil_work_gateway_t *state);
void hil_work_stop(hil_work_gateway_t *state);
bool hil_work_feedback(hil_work_gateway_t *state, uint16_t id,
                       const uint8_t bytes[8], uint32_t now);
bool hil_work_ready(const hil_work_gateway_t *state, unsigned channel, uint32_t now);
uint8_t hil_work_accept(hil_work_gateway_t *state, const uint8_t bytes[8], uint32_t now);
void hil_work_encode(hil_work_gateway_t *state, uint32_t now, uint8_t bytes[8]);
void hil_work_command(unsigned device, uint16_t value, uint8_t sequence,
                      bool enabled, uint8_t bytes[8]);

#ifdef __cplusplus
}
#endif
#endif
