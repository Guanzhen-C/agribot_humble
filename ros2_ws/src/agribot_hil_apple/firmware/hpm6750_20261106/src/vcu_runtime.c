#include "vcu_runtime.h"
#include "vcu_machine_profile.h"
#include <string.h>

static bool motion_profile(uint16_t id)
{
    return id == VCU_MACHINE_DIFFERENTIAL || id == VCU_MACHINE_FOUR_WHEEL
        || id == VCU_MACHINE_MECANUM || id == VCU_MACHINE_OMNI;
}
void vcu_runtime_init(vcu_runtime_t *s, uint16_t machine_id, uint8_t sequence_bits)
{
    memset(s, 0, sizeof(*s)); s->machine_id = machine_id;
    s->sequence_bits = sequence_bits;
    /* Existing adapter IDs stay valid; missing DBC/geometry never implies ready. */
    if (!motion_profile(machine_id)) { s->fault = VCU_FAULT_PROFILE; }
}
void vcu_runtime_reject(vcu_runtime_t *s)
{
    ++s->rejected; s->enabled = false; s->primed = false; s->fault |= VCU_FAULT_COMMAND;
}
void vcu_runtime_tick(vcu_runtime_t *s, uint32_t now)
{
    if (s->has_command && now - s->command_at >= FGCP_VALID_MS) {
        s->enabled = false; s->primed = false; s->fault |= VCU_FAULT_TIMEOUT;
    }
    if (!s->has_feedback || now - s->feedback_at > 300U) {
        s->enabled = false; s->primed = false; s->fault |= VCU_FAULT_FEEDBACK;
    }
}
bool vcu_runtime_accept(vcu_runtime_t *s, const fgcp_command_t *command, uint32_t now)
{
    vcu_runtime_tick(s, now);
    bool stopped = (command->flags & 1U) == 0 || (command->flags & 2U) != 0;
    uint16_t mask = s->sequence_bits == 8 ? 0xffU : 0xffffU;
    uint16_t delta = (command->sequence - s->command.sequence) & mask;
    bool expired = !s->has_command || now - s->command_at >= FGCP_VALID_MS;
    if (s->has_command && !(expired && stopped) && (delta == 0 || delta > mask / 2U)) {
        vcu_runtime_reject(s); s->fault |= VCU_FAULT_SEQUENCE; return false;
    }
    if (s->sequence_bits == 16 && s->has_command && !expired) {
        uint32_t dt = command->source_time_ms - s->command.source_time_ms;
        if (dt == 0 || dt > INT32_MAX) { vcu_runtime_reject(s); s->fault |= VCU_FAULT_SEQUENCE; return false; }
    }
    if (command->fan || command->pump || command->shredder_x100 || (command->flags & 4U)) {
        vcu_runtime_reject(s); return false; /* Motion-only adapter cannot apply implement outputs. */
    }
    s->command = *command; s->command_at = now; s->has_command = true; ++s->accepted;
    s->fault &= (VCU_FAULT_PROFILE | VCU_FAULT_IO | VCU_FAULT_FEEDBACK);
    if (stopped) { s->enabled = false; s->primed = true; }
    else { s->enabled = s->primed && s->fault == 0 && !s->feedback.drive_disabled; }
    return true;
}
bool vcu_runtime_output(vcu_runtime_t *s, uint32_t now, uint16_t *id, uint8_t data[8])
{
    vcu_runtime_tick(s, now);
    if (!motion_profile(s->machine_id)) { return false; }
    hechuan_velocity_t v = {0};
    if (s->enabled && s->fault == 0) { v.forward_mm_s = s->command.linear_mm_s; v.yaw_mrad_s = s->command.angular_mrad_s; }
    *id = HECHUAN_CAN_COMMAND_ID; hechuan_encode_velocity_can(&v, data); return true;
}
bool vcu_runtime_feedback(vcu_runtime_t *s, uint16_t id, const uint8_t data[8], uint32_t now)
{
    if (!motion_profile(s->machine_id) || id < 0x101 || id > 0x103) { return false; }
    if (id == 0x101) { s->feedback_part = 0; s->feedback_started = now; }
    if (id != 0x101U + s->feedback_part || now - s->feedback_started > 100U) {
        s->feedback_part = 0; return false;
    }
    memcpy(s->feedback_bytes + 8U * s->feedback_part, data, 8);
    if (++s->feedback_part != 3) { return false; }
    s->feedback_part = 0;
    if (!hechuan_decode_feedback(s->feedback_bytes, 24, &s->feedback)) { return false; }
    s->feedback_at = now; s->has_feedback = true; s->fault &= ~VCU_FAULT_FEEDBACK;
    if (s->feedback.drive_disabled) { s->enabled = false; }
    return true;
}
