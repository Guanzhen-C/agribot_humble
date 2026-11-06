#include "vcu_telemetry.h"
#include "vcfg_protocol.h"
#include <string.h>
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }
bool vcu_decode_query(const uint8_t frame[24], uint8_t opcode, uint32_t *sequence)
{
    if (memcmp(frame, "VCFG", 4) != 0 || frame[4] != 1 || frame[5] != opcode
        || frame[6] != 8 || frame[7] != 0 || get32(frame + 20) != vcfg_crc32(frame, 20)) { return false; }
    for (size_t i = 12; i < 20; ++i) { if (frame[i] != 0) { return false; } }
    *sequence = get32(frame + 8); return true;
}
void vcu_encode_telemetry(const vcu_runtime_t *s, uint16_t protocol,
    uint8_t compute_interface, bool configured, bool hardware_ready,
    uint32_t now, uint32_t sequence, uint8_t frame[VCU_TELEMETRY_SIZE])
{
    memset(frame, 0, VCU_TELEMETRY_SIZE); memcpy(frame, "VCFG", 4);
    frame[4] = 1; frame[5] = VCU_TELEMETRY_OPCODE; put16(frame + 6, 64); put32(frame + 8, sequence);
    uint8_t *p = frame + 12;
    bool input_fresh = s->has_command && now - s->command_at < FGCP_VALID_MS;
    bool feedback_fresh = s->has_feedback && now - s->feedback_at <= 300;
    p[0] = 1; /* Telemetry schema, independent of configuration schema. */
    p[1] = (configured ? 1 : 0) | (input_fresh ? 2 : 0) | (feedback_fresh ? 4 : 0)
        | (hardware_ready ? 8 : 0) | (s->enabled && hardware_ready ? 16 : 0);
    put16(p + 2, s->fault); put16(p + 4, s->machine_id); put16(p + 6, protocol); p[8] = compute_interface;
    /* Validity: bit0 voltage, bit1 motion, bit2 SOC, bit3 implement feedback. */
    put16(p + 10, feedback_fresh ? 3 : 0); put32(p + 12, now);
    put32(p + 16, s->has_command ? now - s->command_at : UINT32_MAX);
    put32(p + 20, s->has_feedback ? now - s->feedback_at : UINT32_MAX);
    if (feedback_fresh) {
        put32(p + 24, (uint32_t)(int32_t)s->feedback.voltage_mv);
        put32(p + 28, (uint32_t)(int32_t)s->feedback.velocity.forward_mm_s);
        put32(p + 32, (uint32_t)(int32_t)s->feedback.velocity.yaw_mrad_s);
        p[49] = s->feedback.drive_disabled ? 1 : 0;
    }
    if (input_fresh) {
        put32(p + 36, (uint32_t)(int32_t)s->command.linear_mm_s);
        put32(p + 40, (uint32_t)(int32_t)s->command.angular_mrad_s);
        p[46] = s->command.fan; p[47] = s->command.pump; p[48] = s->command.flags;
        put16(p + 50, s->command.shredder_x100);
    }
    put16(p + 44, UINT16_MAX); /* No reported SOC; never infer a battery percentage. */
    put32(p + 52, s->accepted); put32(p + 56, s->rejected);
    put32(frame + 76, vcfg_crc32(frame, 76));
}
void vcu_encode_bridge_telemetry(const vcu_bridge_t *b, uint16_t machine,
    uint16_t protocol, uint8_t compute_interface, bool configured, bool hardware_ready,
    uint32_t now, uint32_t sequence, uint8_t frame[VCU_TELEMETRY_SIZE])
{
    memset(frame, 0, VCU_TELEMETRY_SIZE); memcpy(frame, "VCFG", 4);
    frame[4] = 1; frame[5] = VCU_TELEMETRY_OPCODE; put16(frame + 6, 64); put32(frame + 8, sequence);
    uint8_t *p = frame + 12;
    uint32_t limit = b->profile ? b->profile->fresh_ms : 0U;
    bool command_fresh = vcu_bridge_command_fresh(b, now);
    bool feedback_fresh = vcu_bridge_feedback_fresh(b, now);
    p[0] = 2; /* Native bridge schema; never reports a VCU-owned output enable. */
    p[1] = (configured ? 1 : 0) | (command_fresh ? 2 : 0) | (feedback_fresh ? 4 : 0) | (hardware_ready ? 8 : 0);
    put16(p + 2, vcu_bridge_fault(b, now)); put16(p + 4, machine); put16(p + 6, protocol); p[8] = compute_interface;
    /* Validity: bit0 voltage, bit1 speed/yaw, bit2 left rpm, bit3 right rpm. */
    bool voltage = b->has_voltage && now - b->voltage_at <= limit;
    bool motion = b->has_motion && now - b->motion_at <= limit;
    bool left = b->has_left && now - b->left_at <= limit;
    bool right = b->has_right && now - b->right_at <= limit;
    bool state = b->has_state && now - b->state_at <= limit;
    put16(p + 10, (voltage ? 1 : 0) | (motion ? 2 : 0) | (left ? 4 : 0) | (right ? 8 : 0));
    put32(p + 12, now);
    put32(p + 16, b->has_command ? now - b->command_at : UINT32_MAX);
    put32(p + 20, b->has_feedback ? now - b->feedback_at : UINT32_MAX);
    if (voltage) { put32(p + 24, (uint32_t)b->voltage_mv); }
    if (motion) { put32(p + 28, (uint32_t)b->speed_mm_s); put32(p + 32, (uint32_t)b->yaw_mrad_s); }
    if (command_fresh) { put32(p + 36, (uint32_t)b->target[0]); put32(p + 40, (uint32_t)b->target[1]); }
    if (left) { put16(p + 44, (uint16_t)b->left_rpm); }
    if (right) { put16(p + 46, (uint16_t)b->right_rpm); }
    p[48] = state ? b->work_mode : 0xffU;
    p[49] = state ? b->drive_disabled : 0xffU;
    put32(p + 52, b->command_forwarded); put32(p + 56, b->dropped_frames); put32(p + 60, b->feedback_forwarded);
    put32(frame + 76, vcfg_crc32(frame, 76));
}
void vcu_encode_identity(uint16_t machine, uint16_t protocol, bool hardware_ready,
    bool native_bridge, uint32_t sequence, uint8_t frame[VCU_IDENTITY_SIZE])
{
    memset(frame, 0, VCU_IDENTITY_SIZE); memcpy(frame, "VCFG", 4);
    frame[4] = 1; frame[5] = VCU_IDENTITY_OPCODE; put16(frame + 6, 8); put32(frame + 8, sequence);
    frame[12] = VCU_FIRMWARE_VERSION_MAJOR; frame[13] = VCU_FIRMWARE_VERSION_MINOR;
    frame[14] = VCU_FIRMWARE_VERSION_PATCH;
    frame[15] = (hardware_ready ? 1 : 0) | (native_bridge ? 2 : 0);
    put16(frame + 16, machine); put16(frame + 18, protocol);
    put32(frame + 20, vcfg_crc32(frame, 20));
}
