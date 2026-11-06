#ifndef VCU_MACHINE_PROFILE_H
#define VCU_MACHINE_PROFILE_H

#include "vcfg_protocol.h"

#define VCU_MACHINE_TRACKED 0x0001U
#define VCU_MACHINE_DIFFERENTIAL 0x0010U
#define VCU_MACHINE_FOUR_WHEEL 0x0011U
#define VCU_MACHINE_STEERING 0x0012U
#define VCU_MACHINE_MECANUM 0x0013U
#define VCU_MACHINE_OMNI 0x0014U
#define VCU_PROTOCOL_IMPLEMENT 0x0101U
#define VCU_PROTOCOL_MOTION 0x0201U
#define VCU_PROTOCOL_STEERING 0x0202U

vcfg_status_t vcu_validate_selection(const vcfg_write_request_t *request);
uint32_t vcu_machine_can_bitrate(uint16_t machine_id);

#endif
