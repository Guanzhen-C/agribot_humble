#ifndef FGMP_PROFILE_H
#define FGMP_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FGMP_HEADER_SIZE 128U
#define FGMP_MESSAGE_SIZE 176U
#define FGMP_MAX_PACKAGE 16384U
#define FGMP_MAX_MESSAGES 16U
#define FGMP_MAX_FIELDS 8U
#define FGMP_MAX_FEEDBACK 16U
#define FGMP_MAX_REQUIRED 8U
#define FGMP_SOURCE_COUNT 10U
#define FGMP_TX_PERIOD_MS 100U

typedef enum {
    FGMP_OK, FGMP_BAD_PACKAGE, FGMP_UNSUPPORTED, FGMP_BAD_FRAME,
    FGMP_RANGE, FGMP_SEQUENCE, FGMP_INTERLOCK, FGMP_TIMEOUT, FGMP_IO
} fgmp_status_t;

typedef struct {
    uint8_t source, start_bit, bit_length, flags;
    int32_t numerator, offset, safe_raw;
    uint32_t denominator;
} fgmp_field_t;

typedef struct {
    uint16_t id;
    uint8_t domain, counter, checksum, field_count, base[8];
    fgmp_field_t fields[FGMP_MAX_FIELDS];
} fgmp_message_t;

typedef struct {
    uint8_t uuid[16];
    uint32_t revision, upstream_bitrate, downstream_bitrate, source_offset, source_length;
    uint8_t machine_type, mode, commissioned, model, message_count, feedback_count, required_count;
    uint16_t command_timeout_ms, feedback_timeout_ms;
    uint16_t track_mm, wheelbase_mm, max_wheel_mm_s, max_steering_mrad, min_moving_mm_s;
    uint16_t feedback_ids[FGMP_MAX_FEEDBACK], required_ids[FGMP_MAX_REQUIRED];
    fgmp_message_t messages[FGMP_MAX_MESSAGES];
} fgmp_profile_t;

typedef struct {
    uint32_t id;
    uint8_t length, data[8];
    bool extended, remote, fd;
} fgmp_can_frame_t;

typedef struct {
    fgmp_profile_t profile;
    int32_t values[2][FGMP_SOURCE_COUNT];
    uint32_t command_at[2], feedback_at[FGMP_MAX_REQUIRED], last_tx;
    uint8_t rx_counter[2], feedback_counter[FGMP_MAX_REQUIRED], tx_counter[FGMP_MAX_MESSAGES];
    bool have_command[2], primed[2], feedback_seen[FGMP_MAX_REQUIRED];
    bool initialized, permit, automatic, emergency, tx_started;
    fgmp_status_t fault;
} fgmp_engine_t;

/* Return false on transmit failure. upstream=true means CAN1; false means CAN0. */
typedef bool (*fgmp_send_fn)(void *context, bool upstream, uint16_t id, const uint8_t data[8]);

/* Portable, allocation-free core; no physical port or Flash writes are performed here. */
fgmp_status_t fgmp_profile_decode(const uint8_t *bytes, size_t length, fgmp_profile_t *out);
fgmp_status_t fgmp_engine_init(fgmp_engine_t *engine, const uint8_t *bytes, size_t length);
void fgmp_engine_set_permit(fgmp_engine_t *engine, bool permit);
fgmp_status_t fgmp_engine_receive(fgmp_engine_t *engine, bool from_upstream,
    const fgmp_can_frame_t *frame, uint32_t now, fgmp_send_fn send, void *context);
fgmp_status_t fgmp_engine_tick(fgmp_engine_t *engine, uint32_t now, fgmp_send_fn send, void *context);
fgmp_status_t fgmp_resolve_drive(const fgmp_profile_t *profile, int32_t values[FGMP_SOURCE_COUNT]);
fgmp_status_t fgmp_encode_message(const fgmp_message_t *message,
    const int32_t values[FGMP_SOURCE_COUNT], bool safe, uint8_t counter, uint8_t data[8]);

#endif
