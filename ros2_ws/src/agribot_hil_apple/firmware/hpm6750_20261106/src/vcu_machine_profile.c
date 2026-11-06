#include "vcu_machine_profile.h"

static uint16_t protocol_for_machine(uint16_t machine_id)
{
    switch (machine_id) {
    case VCU_MACHINE_TRACKED:
        return VCU_PROTOCOL_IMPLEMENT;
    case VCU_MACHINE_STEERING:
        return VCU_PROTOCOL_STEERING;
    case VCU_MACHINE_DIFFERENTIAL:
    case VCU_MACHINE_FOUR_WHEEL:
    case VCU_MACHINE_MECANUM:
    case VCU_MACHINE_OMNI:
        return VCU_PROTOCOL_MOTION;
    default:
        return 0U;
    }
}

vcfg_status_t vcu_validate_selection(const vcfg_write_request_t *request)
{
    uint16_t expected = protocol_for_machine(request->machine_id);
    if (expected == 0U) {
        return VCFG_STATUS_UNSUPPORTED_MACHINE;
    }
    if (request->protocol_id != VCU_PROTOCOL_IMPLEMENT
        && request->protocol_id != VCU_PROTOCOL_MOTION
        && request->protocol_id != VCU_PROTOCOL_STEERING) {
        return VCFG_STATUS_UNSUPPORTED_PROTOCOL;
    }
    if (request->protocol_id != expected) {
        return VCFG_STATUS_INCOMPATIBLE_SELECTION;
    }
    return request->compute_interface_id <= VCFG_COMPUTE_INTERFACE_SOCKETCAN
        ? VCFG_STATUS_OK : VCFG_STATUS_UNSUPPORTED_COMPUTE_INTERFACE;
}

uint32_t vcu_machine_can_bitrate(uint16_t machine_id)
{
    if (protocol_for_machine(machine_id) == 0U) {
        return 0U;
    }
    return machine_id == VCU_MACHINE_TRACKED ? 250000U : 1000000U;
}
