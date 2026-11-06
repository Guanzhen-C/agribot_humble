#ifndef VCFG_PROTOCOL_H
#define VCFG_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VCFG_FRAME_SIZE 24U
#define VCFG_PAYLOAD_SIZE 8U
#define VCFG_VERSION 1U
#define VCFG_OPCODE_WRITE_SELECTION 0x10U
#define VCFG_OPCODE_WRITE_SELECTION_ACK 0x90U
#define VCFG_COMPUTE_INTERFACE_SERIAL 0U
#define VCFG_COMPUTE_INTERFACE_CAN 1U
#define VCFG_COMPUTE_INTERFACE_SOCKETCAN 2U
#define VCFG_COMPUTE_INTERFACE_MASK 0xFFU

typedef enum {
    VCFG_STATUS_OK = 0,
    VCFG_STATUS_BAD_FRAME = 1,
    VCFG_STATUS_UNSUPPORTED_VERSION = 2,
    VCFG_STATUS_UNSUPPORTED_MACHINE = 3,
    VCFG_STATUS_UNSUPPORTED_PROTOCOL = 4,
    VCFG_STATUS_INCOMPATIBLE_SELECTION = 5,
    VCFG_STATUS_VCU_NOT_STANDBY = 6,
    VCFG_STATUS_FLASH_WRITE_FAILED = 7,
    VCFG_STATUS_VERIFY_FAILED = 8,
    VCFG_STATUS_UNSUPPORTED_COMPUTE_INTERFACE = 9,
} vcfg_status_t;

typedef struct {
    uint32_t sequence;
    uint16_t machine_id;
    uint16_t protocol_id;
    uint8_t compute_interface_id;
} vcfg_write_request_t;

typedef struct {
    uint32_t sequence;
    vcfg_status_t status;
    uint8_t active_slot;
    uint16_t generation;
    uint16_t machine_id;
    uint16_t protocol_id;
} vcfg_write_ack_t;

typedef struct {
    uint8_t bytes[VCFG_FRAME_SIZE];
    size_t length;
} vcfg_parser_t;

uint32_t vcfg_crc32(const uint8_t *data, size_t length);
void vcfg_parser_init(vcfg_parser_t *parser);
bool vcfg_parser_feed(vcfg_parser_t *parser, uint8_t byte,
                      uint8_t frame[VCFG_FRAME_SIZE]);
vcfg_status_t vcfg_decode_write_request(
    const uint8_t frame[VCFG_FRAME_SIZE], vcfg_write_request_t *request);
void vcfg_encode_write_ack(const vcfg_write_ack_t *ack,
                           uint8_t frame[VCFG_FRAME_SIZE]);

#endif
