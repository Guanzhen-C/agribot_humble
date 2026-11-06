#ifndef VCU_BRIDGE_H
#define VCU_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

/* Native CAN bridge used by the agribot RDK stack: CAN1 (compute) and CAN0
 * (machine) run the machine's own protocol; only newly received whitelisted
 * frames are forwarded, nothing is held or repeated. */

#define VCU_FIRMWARE_VERSION_MAJOR 1U
#define VCU_FIRMWARE_VERSION_MINOR 0U
#define VCU_FIRMWARE_VERSION_PATCH 0U

typedef enum { VCU_MODE_ADAPTER = 0, VCU_MODE_BRIDGE = 1 } vcu_mode_t;

typedef struct {
    uint16_t protocol_id;
    uint16_t command_id;
    uint16_t feedback_ids[3];
    uint32_t bitrate;
    uint32_t fresh_ms;
} vcu_bridge_profile_t;

typedef struct {
    const vcu_bridge_profile_t *profile;
    bool io_failed;
    bool has_command;
    bool has_feedback;
    uint32_t command_at;
    uint32_t feedback_at;
    int32_t target[2];
    uint32_t voltage_at, motion_at, left_at, right_at, state_at;
    bool has_voltage, has_motion, has_left, has_right, has_state;
    int32_t voltage_mv, speed_mm_s, yaw_mrad_s;
    int16_t left_rpm, right_rpm;
    uint8_t work_mode, drive_disabled;
    uint32_t command_forwarded, dropped_frames, feedback_forwarded;
    uint8_t feedback_bytes[24];
    uint8_t feedback_part;
    uint32_t feedback_started;
} vcu_bridge_t;

const vcu_bridge_profile_t *vcu_bridge_profile(uint16_t protocol_id);
vcu_mode_t vcu_select_mode(uint16_t protocol_id, uint8_t compute_interface);
bool vcu_bridge_init(vcu_bridge_t *bridge, uint16_t protocol_id);
/* Return true when the frame must be forwarded to the other bus. */
bool vcu_bridge_from_compute(vcu_bridge_t *bridge, uint32_t id, const uint8_t *data,
                             bool classic_8_byte, uint32_t now);
bool vcu_bridge_from_machine(vcu_bridge_t *bridge, uint32_t id, const uint8_t *data,
                             bool classic_8_byte, uint32_t now);
void vcu_bridge_sent(vcu_bridge_t *bridge, bool command, bool ok);
bool vcu_bridge_command_fresh(const vcu_bridge_t *bridge, uint32_t now);
bool vcu_bridge_feedback_fresh(const vcu_bridge_t *bridge, uint32_t now);
uint16_t vcu_bridge_fault(const vcu_bridge_t *bridge, uint32_t now);

#endif
