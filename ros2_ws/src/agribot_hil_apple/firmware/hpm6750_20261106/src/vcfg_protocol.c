#include "vcfg_protocol.h"

#include <string.h>

static const uint8_t vcfg_magic[4] = {0x56U, 0x43U, 0x46U, 0x47U};

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U)
        | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

uint32_t vcfg_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = UINT32_MAX;

    for (size_t index = 0; index < length; index++) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8U; bit++) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return crc ^ UINT32_MAX;
}

void vcfg_parser_init(vcfg_parser_t *parser)
{
    parser->length = 0U;
}

bool vcfg_parser_feed(vcfg_parser_t *parser, uint8_t byte,
                      uint8_t frame[VCFG_FRAME_SIZE])
{
    if (parser->length < sizeof(vcfg_magic)) {
        if (byte == vcfg_magic[parser->length]) {
            parser->bytes[parser->length++] = byte;
        } else if (byte == vcfg_magic[0]) {
            parser->bytes[0] = byte;
            parser->length = 1U;
        } else {
            parser->length = 0U;
        }
        return false;
    }

    parser->bytes[parser->length++] = byte;
    if (parser->length != VCFG_FRAME_SIZE) {
        return false;
    }

    memcpy(frame, parser->bytes, VCFG_FRAME_SIZE);
    parser->length = 0U;
    return true;
}

vcfg_status_t vcfg_decode_write_request(
    const uint8_t frame[VCFG_FRAME_SIZE], vcfg_write_request_t *request)
{
    request->sequence = read_u32_le(&frame[8]);
    request->machine_id = read_u16_le(&frame[12]);
    request->protocol_id = read_u16_le(&frame[14]);
    uint32_t flags = read_u32_le(&frame[16]);
    request->compute_interface_id = (uint8_t)(flags
                                               & VCFG_COMPUTE_INTERFACE_MASK);

    if (memcmp(frame, vcfg_magic, sizeof(vcfg_magic)) != 0) {
        return VCFG_STATUS_BAD_FRAME;
    }
    if (read_u32_le(&frame[20]) != vcfg_crc32(frame, 20U)) {
        return VCFG_STATUS_BAD_FRAME;
    }
    if (frame[4] != VCFG_VERSION) {
        return VCFG_STATUS_UNSUPPORTED_VERSION;
    }
    if (frame[5] != VCFG_OPCODE_WRITE_SELECTION
        || read_u16_le(&frame[6]) != VCFG_PAYLOAD_SIZE
        || (flags & ~VCFG_COMPUTE_INTERFACE_MASK) != 0U) {
        return VCFG_STATUS_BAD_FRAME;
    }
    if (request->compute_interface_id > VCFG_COMPUTE_INTERFACE_SOCKETCAN) {
        return VCFG_STATUS_UNSUPPORTED_COMPUTE_INTERFACE;
    }
    return VCFG_STATUS_OK;
}

void vcfg_encode_write_ack(const vcfg_write_ack_t *ack,
                           uint8_t frame[VCFG_FRAME_SIZE])
{
    memcpy(frame, vcfg_magic, sizeof(vcfg_magic));
    frame[4] = VCFG_VERSION;
    frame[5] = VCFG_OPCODE_WRITE_SELECTION_ACK;
    write_u16_le(&frame[6], VCFG_PAYLOAD_SIZE);
    write_u32_le(&frame[8], ack->sequence);
    frame[12] = (uint8_t)ack->status;
    frame[13] = ack->active_slot;
    write_u16_le(&frame[14], ack->generation);
    write_u16_le(&frame[16], ack->machine_id);
    write_u16_le(&frame[18], ack->protocol_id);
    write_u32_le(&frame[20], vcfg_crc32(frame, 20U));
}
