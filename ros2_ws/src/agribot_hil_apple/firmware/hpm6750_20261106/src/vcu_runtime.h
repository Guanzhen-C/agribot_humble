#ifndef VCU_RUNTIME_H
#define VCU_RUNTIME_H
#include "fgcp_protocol.h"
#include "hechuan_motion.h"

enum { VCU_FAULT_PROFILE = 1, VCU_FAULT_TIMEOUT = 2, VCU_FAULT_SEQUENCE = 4,
    VCU_FAULT_COMMAND = 8, VCU_FAULT_FEEDBACK = 16, VCU_FAULT_IO = 32 };
typedef struct {
    uint16_t machine_id;
    uint16_t fault;
    bool has_command;
    bool has_feedback;
    bool primed;
    bool enabled;
    uint8_t sequence_bits;
    uint32_t command_at;
    uint32_t feedback_at;
    uint32_t accepted;
    uint32_t rejected;
    fgcp_command_t command;
    hechuan_feedback_t feedback;
    uint8_t feedback_bytes[24];
    uint8_t feedback_part;
    uint32_t feedback_started;
} vcu_runtime_t;
void vcu_runtime_init(vcu_runtime_t *state, uint16_t machine_id, uint8_t sequence_bits);
bool vcu_runtime_accept(vcu_runtime_t *state, const fgcp_command_t *command, uint32_t now);
void vcu_runtime_reject(vcu_runtime_t *state);
void vcu_runtime_tick(vcu_runtime_t *state, uint32_t now);
bool vcu_runtime_output(vcu_runtime_t *state, uint32_t now, uint16_t *id, uint8_t data[8]);
bool vcu_runtime_feedback(vcu_runtime_t *state, uint16_t id, const uint8_t data[8], uint32_t now);
#endif
