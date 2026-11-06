#include "vcu_bridge.h"
#include "hechuan_motion.h"
#include "vcu_machine_profile.h"
#include "vcu_runtime.h"
#include <string.h>

/* WHEELTEC Ackermann: 0x181 speed/front-wheel angle, 0x101..0x103 feedback.
 * Three-in-one tracked: 0x514 track percent, 0x532 chassis, 0x533/0x534 motors. */
static const vcu_bridge_profile_t profiles[] = {
    { VCU_PROTOCOL_STEERING, 0x181U, { 0x101U, 0x102U, 0x103U }, 1000000U, 300U },
    { VCU_PROTOCOL_IMPLEMENT, 0x514U, { 0x532U, 0x533U, 0x534U }, 250000U, 600U },
};

static int16_t i16_be(const uint8_t *p)
{
    uint16_t v = (uint16_t)((uint16_t)p[0] << 8U) | p[1];
    return (int16_t)(v <= INT16_MAX ? (int32_t)v : (int32_t)v - 65536);
}
static int16_t i16_le(const uint8_t *p) { const uint8_t be[2] = { p[1], p[0] }; return i16_be(be); }
static int8_t i8(uint8_t v) { return (int8_t)(v < 0x80U ? (int16_t)v : (int16_t)v - 256); }
static bool xor_ok(const uint8_t *d)
{
    uint8_t x = 0U;
    for (unsigned i = 0; i < 7U; ++i) { x ^= d[i]; }
    return x == d[7];
}
static bool fresh(bool has, uint32_t at, uint32_t now, uint32_t limit) { return has && now - at <= limit; }

const vcu_bridge_profile_t *vcu_bridge_profile(uint16_t protocol_id)
{
    for (unsigned i = 0; i < sizeof(profiles) / sizeof(profiles[0]); ++i) {
        if (profiles[i].protocol_id == protocol_id) { return &profiles[i]; }
    }
    return NULL;
}

vcu_mode_t vcu_select_mode(uint16_t protocol_id, uint8_t compute_interface)
{
    bool can = compute_interface == VCFG_COMPUTE_INTERFACE_CAN
        || compute_interface == VCFG_COMPUTE_INTERFACE_SOCKETCAN;
    return can && vcu_bridge_profile(protocol_id) ? VCU_MODE_BRIDGE : VCU_MODE_ADAPTER;
}

bool vcu_bridge_init(vcu_bridge_t *b, uint16_t protocol_id)
{
    memset(b, 0, sizeof(*b));
    b->profile = vcu_bridge_profile(protocol_id);
    b->work_mode = 0xffU; b->drive_disabled = 0xffU;
    return b->profile != NULL;
}

bool vcu_bridge_from_compute(vcu_bridge_t *b, uint32_t id, const uint8_t *data,
                             bool classic_8_byte, uint32_t now)
{
    if (!b->profile || id != b->profile->command_id) { return false; }
    if (!classic_8_byte) { ++b->dropped_frames; return false; }
    if (b->profile->protocol_id == VCU_PROTOCOL_STEERING) {
        b->target[0] = i16_be(data); b->target[1] = i16_be(data + 4);  /* mm/s, front-wheel mrad */
    } else {
        b->target[0] = i8(data[1]); b->target[1] = i8(data[2]);  /* left/right track percent */
    }
    b->has_command = true; b->command_at = now;
    return true;
}

static void ackermann_feedback(vcu_bridge_t *b, uint32_t id, const uint8_t *data, uint32_t now)
{
    if (id == 0x101U) { b->feedback_part = 0; b->feedback_started = now; }
    if (id != 0x101U + b->feedback_part || now - b->feedback_started > 100U) { b->feedback_part = 0; return; }
    memcpy(b->feedback_bytes + 8U * b->feedback_part, data, 8);
    if (++b->feedback_part != 3U) { return; }
    b->feedback_part = 0;
    hechuan_feedback_t f;
    if (!hechuan_decode_feedback(b->feedback_bytes, sizeof(b->feedback_bytes), &f)) { return; }
    b->voltage_mv = f.voltage_mv; b->has_voltage = true; b->voltage_at = now;
    b->speed_mm_s = f.velocity.forward_mm_s; b->yaw_mrad_s = f.velocity.yaw_mrad_s;
    b->has_motion = true; b->motion_at = now;
    b->drive_disabled = f.drive_disabled ? 1U : 0U; b->has_state = true; b->state_at = now;
}

static void tracked_feedback(vcu_bridge_t *b, uint32_t id, const uint8_t *data, uint32_t now)
{
    if (!xor_ok(data)) { return; }
    if (id == 0x532U) {
        b->voltage_mv = (int32_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8U)) * 100;  /* 0.1 V */
        b->has_voltage = true; b->voltage_at = now;
        b->work_mode = data[0] & 0x03U; b->drive_disabled = (data[0] >> 2U) & 1U;  /* emergency stop */
        b->has_state = true; b->state_at = now;
    } else if (id == 0x533U) {
        b->left_rpm = i16_le(data + 1); b->has_left = true; b->left_at = now;
    } else {
        b->right_rpm = i16_le(data + 1); b->has_right = true; b->right_at = now;
    }
}

bool vcu_bridge_from_machine(vcu_bridge_t *b, uint32_t id, const uint8_t *data,
                             bool classic_8_byte, uint32_t now)
{
    if (!b->profile) { return false; }
    bool listed = false;
    for (unsigned i = 0; i < 3U; ++i) { listed = listed || id == b->profile->feedback_ids[i]; }
    if (!listed) { return false; }
    if (!classic_8_byte) { ++b->dropped_frames; return false; }
    b->has_feedback = true; b->feedback_at = now;
    if (b->profile->protocol_id == VCU_PROTOCOL_STEERING) { ackermann_feedback(b, id, data, now); }
    else { tracked_feedback(b, id, data, now); }
    return true;
}

void vcu_bridge_sent(vcu_bridge_t *b, bool command, bool ok)
{
    b->io_failed = !ok;
    if (!ok) { ++b->dropped_frames; }
    else if (command) { ++b->command_forwarded; }
    else { ++b->feedback_forwarded; }
}

bool vcu_bridge_command_fresh(const vcu_bridge_t *b, uint32_t now)
{
    return b->profile && fresh(b->has_command, b->command_at, now, b->profile->fresh_ms);
}

bool vcu_bridge_feedback_fresh(const vcu_bridge_t *b, uint32_t now)
{
    return b->profile && fresh(b->has_feedback, b->feedback_at, now, b->profile->fresh_ms);
}

uint16_t vcu_bridge_fault(const vcu_bridge_t *b, uint32_t now)
{
    if (!b->profile) { return VCU_FAULT_PROFILE; }
    uint16_t fault = b->io_failed ? VCU_FAULT_IO : 0U;
    if (b->has_command && !vcu_bridge_command_fresh(b, now)) { fault |= VCU_FAULT_TIMEOUT; }
    if (!vcu_bridge_feedback_fresh(b, now)) { fault |= VCU_FAULT_FEEDBACK; }
    return fault;
}
