#include "fgmp_profile.h"
#include "vcfg_protocol.h"
#include <math.h>
#include <string.h>

static uint16_t get16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)get16(p) | ((uint32_t)get16(p + 2) << 16); }
static int32_t signed32(const uint8_t *p)
{
    uint32_t value = get32(p);
    return value <= INT32_MAX ? (int32_t)value : (int32_t)((int64_t)value - 4294967296LL);
}
static bool contains(const uint16_t *values, size_t count, uint16_t value)
{
    for (size_t i = 0; i < count; i++) { if (values[i] == value) { return true; } }
    return false;
}
static bool zero_bytes(const uint8_t *p, size_t length)
{
    for (size_t i = 0; i < length; i++) { if (p[i]) { return false; } }
    return true;
}
static bool bitrate_valid(uint32_t rate)
{
    return rate == 125000 || rate == 250000 || rate == 500000 || rate == 1000000;
}
static bool raw_fits(const fgmp_field_t *f, int64_t value)
{
    int64_t min = (f->flags & 1U) ? -(1LL << (f->bit_length - 1U)) : 0;
    int64_t max = (1LL << (f->bit_length - ((f->flags & 1U) ? 1U : 0U))) - 1;
    return value >= min && value <= max;
}

fgmp_status_t fgmp_profile_decode(const uint8_t *bytes, size_t length, fgmp_profile_t *out)
{
    static const uint16_t known[] = {0x500,0x501,0x532,0x533,0x534,0x5b0,0x5b2,0x5b3,0x5b4,0x650,0x652};
    if (!out) { return FGMP_BAD_PACKAGE; }
    memset(out, 0, sizeof(*out));
    if (!bytes || length < FGMP_HEADER_SIZE || length > FGMP_MAX_PACKAGE
        || memcmp(bytes, "FGMP", 4) || get16(bytes + 4) != 1 || get16(bytes + 6) != FGMP_HEADER_SIZE
        || get32(bytes + 8) != length || get32(bytes + 12) != vcfg_crc32(bytes + 16, length - 16)) {
        return FGMP_BAD_PACKAGE;
    }
    fgmp_profile_t p = {0};
    memcpy(p.uuid, bytes + 16, 16); p.revision = get32(bytes + 32);
    p.machine_type = bytes[36]; p.mode = bytes[37]; p.commissioned = bytes[38];
    p.upstream_bitrate = get32(bytes + 40); p.downstream_bitrate = get32(bytes + 44);
    p.command_timeout_ms = get16(bytes + 48); p.feedback_timeout_ms = get16(bytes + 50);
    p.track_mm = get16(bytes + 52); p.wheelbase_mm = get16(bytes + 54);
    p.max_wheel_mm_s = get16(bytes + 56); p.max_steering_mrad = get16(bytes + 58); p.min_moving_mm_s = get16(bytes + 60);
    p.model = bytes[62]; p.message_count = bytes[63]; p.feedback_count = bytes[64]; p.required_count = bytes[65];
    p.source_length = get32(bytes + 116); p.source_offset = get32(bytes + 120);
    if (!p.revision || p.machine_type > 2 || p.mode > 1 || p.model > 1 || p.commissioned > 1 || bytes[39] != 1
        || !zero_bytes(bytes + 66, 2) || !zero_bytes(bytes + 124, 4)
        || !bitrate_valid(p.upstream_bitrate) || !bitrate_valid(p.downstream_bitrate)
        || p.command_timeout_ms < 100 || p.command_timeout_ms > 500 || p.feedback_timeout_ms < 200 || p.feedback_timeout_ms > 5000
        || p.message_count > FGMP_MAX_MESSAGES || !p.feedback_count || p.feedback_count > FGMP_MAX_FEEDBACK
        || !p.required_count || p.required_count > FGMP_MAX_REQUIRED
        || p.source_offset != FGMP_HEADER_SIZE + p.message_count * FGMP_MESSAGE_SIZE
        || !p.source_length || p.source_offset > length || p.source_length != length - p.source_offset) {
        return FGMP_BAD_PACKAGE;
    }
    if (p.model) {
        if (p.track_mm < 100 || p.track_mm > 10000 || p.wheelbase_mm < 100 || p.wheelbase_mm > 10000
            || !p.max_wheel_mm_s || p.max_wheel_mm_s > 10000 || !p.max_steering_mrad || p.max_steering_mrad > 1000
            || !p.min_moving_mm_s || p.min_moving_mm_s > 1000 || p.min_moving_mm_s > p.max_wheel_mm_s) { return FGMP_BAD_PACKAGE; }
    } else if (p.machine_type == 2 || !zero_bytes(bytes + 52, 10)) { return FGMP_BAD_PACKAGE; }
    if ((!p.mode && (p.message_count || p.model)) || (p.mode && !p.message_count)) { return FGMP_BAD_PACKAGE; }
    for (size_t i = 0; i < p.feedback_count; i++) {
        uint16_t id = get16(bytes + 68 + i * 2);
        if (!contains(known, sizeof(known) / sizeof(known[0]), id) || contains(p.feedback_ids, i, id)) { return FGMP_BAD_PACKAGE; }
        p.feedback_ids[i] = id;
    }
    for (size_t i = 0; i < p.required_count; i++) {
        uint16_t id = get16(bytes + 100 + i * 2);
        if (!contains(p.feedback_ids, p.feedback_count, id) || contains(p.required_ids, i, id)) { return FGMP_BAD_PACKAGE; }
        p.required_ids[i] = id;
    }
    if (!contains(p.required_ids, p.required_count, 0x532)
        || !zero_bytes(bytes + 68 + p.feedback_count * 2, (FGMP_MAX_FEEDBACK - p.feedback_count) * 2)
        || !zero_bytes(bytes + 100 + p.required_count * 2, (FGMP_MAX_REQUIRED - p.required_count) * 2)) { return FGMP_BAD_PACKAGE; }
    for (size_t i = 0; i < p.message_count; i++) {
        const uint8_t *raw = bytes + FGMP_HEADER_SIZE + i * FGMP_MESSAGE_SIZE;
        fgmp_message_t *m = &p.messages[i];
        m->id = get16(raw); m->domain = raw[2]; m->counter = raw[3]; m->checksum = raw[4]; m->field_count = raw[5];
        memcpy(m->base, raw + 8, 8);
        if (!m->id || m->id > 0x7ff || m->domain > 1 || m->counter > 1 || m->checksum > 1
            || !m->field_count || m->field_count > FGMP_MAX_FIELDS || !zero_bytes(raw + 6, 2)
            || contains(p.feedback_ids, p.feedback_count, m->id)) { return FGMP_BAD_PACKAGE; }
        for (size_t j = 0; j < i; j++) { if (p.messages[j].id == m->id) { return FGMP_BAD_PACKAGE; } }
        uint64_t occupied = (m->counter ? (15ULL << 48) : 0) | (m->checksum ? (255ULL << 56) : 0);
        for (size_t j = 0; j < m->field_count; j++) {
            const uint8_t *fraw = raw + 16 + j * 20;
            fgmp_field_t *f = &m->fields[j];
            f->source = fraw[0]; f->start_bit = fraw[1]; f->bit_length = fraw[2]; f->flags = fraw[3];
            f->numerator = signed32(fraw + 4); f->denominator = get32(fraw + 8);
            f->offset = signed32(fraw + 12); f->safe_raw = signed32(fraw + 16);
            if (f->source >= FGMP_SOURCE_COUNT || f->flags > 3 || !f->bit_length || f->bit_length > 16
                || f->start_bit + f->bit_length > 64 || !f->numerator || f->numerator < -100000 || f->numerator > 100000
                || !f->denominator || f->denominator > 100000 || f->offset < -1000000 || f->offset > 1000000
                || ((f->flags & 2U) && (f->start_bit % 8 || (f->bit_length != 8 && f->bit_length != 16)))
                || ((f->source >= 3 && f->source <= 5) != (m->domain == 1))
                || (f->source >= 6 && !p.model) || (f->source >= 8 && p.machine_type != 2)
                || !raw_fits(f, f->safe_raw)) { return FGMP_BAD_PACKAGE; }
            uint64_t mask = ((1ULL << f->bit_length) - 1) << f->start_bit;
            if (occupied & mask) { return FGMP_BAD_PACKAGE; }
            occupied |= mask;
        }
        if (!zero_bytes(raw + 16 + m->field_count * 20, (FGMP_MAX_FIELDS - m->field_count) * 20)) { return FGMP_BAD_PACKAGE; }
    }
    *out = p;
    return FGMP_OK;
}

fgmp_status_t fgmp_resolve_drive(const fgmp_profile_t *p, int32_t values[FGMP_SOURCE_COUNT])
{
    if (!p->model) { return FGMP_OK; }
    double left = values[0] * p->max_wheel_mm_s / 100.0;
    double right = values[1] * p->max_wheel_mm_s / 100.0;
    double forward = (left + right) / 2.0, yaw = (right - left) / p->track_mm;
    double left_angle = 0, right_angle = 0;
    if (p->machine_type == 2) {
        if (fabs(forward) < p->min_moving_mm_s) {
            if (fabs(yaw) > 1e-9) { return FGMP_UNSUPPORTED; }
            forward = 0; yaw = 0;
        } else {
            double tangent = tan(p->max_steering_mrad / 1000.0);
            double limit = tangent / (p->wheelbase_mm + p->track_mm / 2.0 * tangent);
            double curvature = fmax(-limit, fmin(limit, yaw / forward));
            yaw = curvature * forward;
            left_angle = atan(p->wheelbase_mm * curvature / (1 - p->track_mm * curvature / 2.0));
            right_angle = atan(p->wheelbase_mm * curvature / (1 + p->track_mm * curvature / 2.0));
        }
    }
    values[6] = (int32_t)forward; values[7] = (int32_t)(yaw * 1000);
    values[8] = (int32_t)(left_angle * 1000); values[9] = (int32_t)(right_angle * 1000);
    return FGMP_OK;
}

static uint8_t checksum(const uint8_t data[8])
{
    uint8_t value = 0;
    for (size_t i = 0; i < 7; i++) { value ^= data[i]; }
    return value;
}
fgmp_status_t fgmp_encode_message(const fgmp_message_t *m, const int32_t values[FGMP_SOURCE_COUNT],
    bool safe, uint8_t counter, uint8_t data[8])
{
    uint8_t encoded[8]; memcpy(encoded, m->base, 8);
    for (size_t i = 0; i < m->field_count; i++) {
        const fgmp_field_t *f = &m->fields[i];
        int64_t raw = safe ? f->safe_raw : (int64_t)values[f->source] * f->numerator / f->denominator + f->offset;
        if (!raw_fits(f, raw)) { return FGMP_RANGE; }
        uint32_t bits = (uint32_t)raw & ((1U << f->bit_length) - 1);
        for (unsigned bit = 0; bit < f->bit_length; bit++) {
            unsigned target = f->start_bit + bit;
            if (f->flags & 2U) { target = f->start_bit + (f->bit_length / 8U - 1U - bit / 8U) * 8U + bit % 8U; }
            encoded[target / 8] = (uint8_t)((encoded[target / 8] & ~(1U << (target % 8))) | (((bits >> bit) & 1U) << (target % 8)));
        }
    }
    if (m->counter) { encoded[6] = (encoded[6] & 0xf0U) | (counter & 15U); }
    if (m->checksum) { encoded[7] = checksum(encoded); }
    memcpy(data, encoded, 8);
    return FGMP_OK;
}

static void disarm(fgmp_engine_t *e, fgmp_status_t fault)
{
    e->primed[0] = false; e->primed[1] = false; e->fault = fault;
}
fgmp_status_t fgmp_engine_init(fgmp_engine_t *e, const uint8_t *bytes, size_t length)
{
    if (!e) { return FGMP_BAD_PACKAGE; }
    memset(e, 0, sizeof(*e));
    e->fault = fgmp_profile_decode(bytes, length, &e->profile);
    e->initialized = e->fault == FGMP_OK;
    return e->fault;
}
void fgmp_engine_set_permit(fgmp_engine_t *e, bool permit)
{
    if (!e) { return; }
    bool next = permit && e->initialized && e->profile.commissioned;
    if (!next || next != e->permit) { disarm(e, FGMP_INTERLOCK); }
    e->permit = next;
}
static bool healthy(const fgmp_engine_t *e, uint32_t now)
{
    if (!e->permit || !e->automatic || e->emergency) { return false; }
    for (size_t i = 0; i < e->profile.required_count; i++) {
        if (!e->feedback_seen[i] || now - e->feedback_at[i] >= e->profile.feedback_timeout_ms) { return false; }
    }
    return true;
}
static fgmp_status_t transmit(fgmp_engine_t *e, fgmp_send_fn send, void *context, bool upstream, uint16_t id, const uint8_t data[8])
{
    if (!send || !send(context, upstream, id, data)) {
        disarm(e, FGMP_IO); e->permit = false; return FGMP_IO;
    }
    return FGMP_OK;
}
fgmp_status_t fgmp_engine_receive(fgmp_engine_t *e, bool from_upstream,
    const fgmp_can_frame_t *frame, uint32_t now, fgmp_send_fn send, void *context)
{
    if (!e || !e->initialized || !frame) { return FGMP_BAD_PACKAGE; }
    bool interested = from_upstream ? (frame->id == 0x514 || frame->id == 0x580)
        : (frame->id <= 0x7ff && contains(e->profile.feedback_ids, e->profile.feedback_count, (uint16_t)frame->id));
    if (!interested) { return FGMP_UNSUPPORTED; }
    if (frame->length != 8 || frame->extended || frame->remote || frame->fd || checksum(frame->data) != frame->data[7]) {
        disarm(e, FGMP_BAD_FRAME); return FGMP_BAD_FRAME;
    }
    if (!from_upstream) {
        for (size_t i = 0; i < e->profile.required_count; i++) {
            if (e->profile.required_ids[i] != frame->id) { continue; }
            uint8_t delta = ((frame->data[6] & 15U) - e->feedback_counter[i]) & 15U;
            if (e->feedback_seen[i] && now - e->feedback_at[i] < e->profile.feedback_timeout_ms && (!delta || delta > 8)) {
                disarm(e, FGMP_SEQUENCE); return FGMP_SEQUENCE;
            }
        }
        if (frame->id == 0x532) {
            e->automatic = (frame->data[0] & 3U) == 1U;
            e->emergency = (frame->data[0] & 4U) != 0;
            if (!e->automatic || e->emergency) { disarm(e, FGMP_INTERLOCK); }
        }
        for (size_t i = 0; i < e->profile.required_count; i++) {
            if (e->profile.required_ids[i] == frame->id) {
                e->feedback_at[i] = now; e->feedback_seen[i] = true; e->feedback_counter[i] = frame->data[6] & 15U;
            }
        }
        return transmit(e, send, context, true, (uint16_t)frame->id, frame->data);
    }
    unsigned domain = frame->id == 0x580 ? 1U : 0U;
    int32_t values[FGMP_SOURCE_COUNT] = {0}; bool neutral;
    if (!domain) {
        values[0] = frame->data[1] < 128 ? frame->data[1] : (int32_t)frame->data[1] - 256;
        values[1] = frame->data[2] < 128 ? frame->data[2] : (int32_t)frame->data[2] - 256;
        values[2] = frame->data[3] & 1U;
        if (values[0] < -100 || values[0] > 100 || values[1] < -100 || values[1] > 100) { disarm(e, FGMP_RANGE); return FGMP_RANGE; }
        neutral = values[0] == 0 && values[1] == 0;
        fgmp_status_t status = fgmp_resolve_drive(&e->profile, values);
        if (status != FGMP_OK) { disarm(e, status); return status; }
    } else {
        values[3] = frame->data[0] & 15U; values[4] = frame->data[0] >> 4; values[5] = get16(frame->data + 1);
        if (values[3] > 8 || values[4] > 8 || values[5] > 10000) { disarm(e, FGMP_RANGE); return FGMP_RANGE; }
        neutral = !values[3] && !values[4] && !values[5];
    }
    uint8_t counter = frame->data[6] & 15U;
    uint8_t delta = (counter - e->rx_counter[domain]) & 15U;
    bool expired = e->have_command[domain] && now - e->command_at[domain] >= e->profile.command_timeout_ms;
    if (expired) { e->primed[domain] = false; }
    if (e->have_command[domain] && !(expired && neutral) && (!delta || delta > 8)) { disarm(e, FGMP_SEQUENCE); return FGMP_SEQUENCE; }
    if (e->profile.mode) {
        bool supported = false;
        for (size_t i = 0; i < e->profile.message_count; i++) {
            if (e->profile.messages[i].domain != domain) { continue; }
            uint8_t output[8]; supported = true;
            fgmp_status_t status = fgmp_encode_message(&e->profile.messages[i], values, false, 0, output);
            if (status != FGMP_OK) { disarm(e, status); return status; }
        }
        if (!supported && !neutral) { disarm(e, FGMP_UNSUPPORTED); return FGMP_UNSUPPORTED; }
    }
    if (!healthy(e, now)) { disarm(e, FGMP_INTERLOCK); }
    else if (neutral) { e->primed[domain] = true; e->fault = FGMP_OK; }
    e->rx_counter[domain] = counter; e->command_at[domain] = now; e->have_command[domain] = true;
    memcpy(e->values[domain], values, sizeof(values));
    if (!e->primed[domain] || !healthy(e, now)) { return FGMP_INTERLOCK; }
    if (!e->profile.mode) { return transmit(e, send, context, false, (uint16_t)frame->id, frame->data); }
    return FGMP_OK;
}
fgmp_status_t fgmp_engine_tick(fgmp_engine_t *e, uint32_t now, fgmp_send_fn send, void *context)
{
    if (!e || !e->initialized) { return FGMP_BAD_PACKAGE; }
    bool ready = healthy(e, now);
    if (!ready) { disarm(e, FGMP_INTERLOCK); }
    for (size_t domain = 0; domain < 2; domain++) {
        if (e->have_command[domain] && now - e->command_at[domain] >= e->profile.command_timeout_ms) {
            e->primed[domain] = false; e->fault = FGMP_TIMEOUT;
        }
    }
    /* Native traffic is never replayed: downstream watchdog sees an actual loss. */
    if (!e->profile.mode || !e->profile.commissioned || (!e->permit && !e->tx_started)) { return e->fault; }
    if (e->tx_started && now - e->last_tx < FGMP_TX_PERIOD_MS) { return e->fault; }
    e->tx_started = true; e->last_tx = now;
    for (size_t i = 0; i < e->profile.message_count; i++) {
        const fgmp_message_t *m = &e->profile.messages[i]; uint8_t data[8];
        bool safe = !ready || !e->primed[m->domain] || !e->have_command[m->domain];
        fgmp_status_t status = fgmp_encode_message(m, e->values[m->domain], safe, e->tx_counter[i], data);
        if (status != FGMP_OK) { disarm(e, status); return status; }
        status = transmit(e, send, context, false, m->id, data);
        if (status != FGMP_OK) { return status; }
        e->tx_counter[i] = (e->tx_counter[i] + 1U) & 15U;
    }
    return e->fault;
}
