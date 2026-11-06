#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "vcfg_protocol.h"
#include "vcu_selection_store.h"

typedef struct {
    uint8_t slots[2][VCU_SELECTION_RECORD_SIZE];
    unsigned int writes;
    bool fail_write;
    bool corrupt_after_write;
} memory_flash_t;

static bool memory_read(void *opaque, uint8_t slot, uint8_t *data,
                        size_t length)
{
    memory_flash_t *flash = opaque;
    memcpy(data, flash->slots[slot], length);
    return true;
}

static bool memory_write(void *opaque, uint8_t slot, const uint8_t *data,
                         size_t length)
{
    memory_flash_t *flash = opaque;
    flash->writes++;
    if (flash->fail_write) {
        return false;
    }
    memset(flash->slots[slot], 0xFF, VCU_SELECTION_RECORD_SIZE);
    memcpy(flash->slots[slot], data, length);
    if (flash->corrupt_after_write) {
        flash->slots[slot][12] ^= 1U;
    }
    return true;
}

static void encode_request(uint32_t sequence, uint16_t machine_id,
                           uint16_t protocol_id, uint8_t compute_interface_id,
                           uint8_t frame[VCFG_FRAME_SIZE])
{
    static const uint8_t magic[4] = {0x56U, 0x43U, 0x46U, 0x47U};
    memset(frame, 0, VCFG_FRAME_SIZE);
    memcpy(frame, magic, sizeof(magic));
    frame[4] = VCFG_VERSION;
    frame[5] = VCFG_OPCODE_WRITE_SELECTION;
    frame[6] = VCFG_PAYLOAD_SIZE;
    frame[8] = (uint8_t)sequence;
    frame[9] = (uint8_t)(sequence >> 8U);
    frame[10] = (uint8_t)(sequence >> 16U);
    frame[11] = (uint8_t)(sequence >> 24U);
    frame[12] = (uint8_t)machine_id;
    frame[13] = (uint8_t)(machine_id >> 8U);
    frame[14] = (uint8_t)protocol_id;
    frame[15] = (uint8_t)(protocol_id >> 8U);
    frame[16] = compute_interface_id;
    uint32_t crc = vcfg_crc32(frame, 20U);
    frame[20] = (uint8_t)crc;
    frame[21] = (uint8_t)(crc >> 8U);
    frame[22] = (uint8_t)(crc >> 16U);
    frame[23] = (uint8_t)(crc >> 24U);
}

static void test_protocol(void)
{
    uint8_t request[VCFG_FRAME_SIZE];
    encode_request(0x12345678U, 0x0001U, 0x0101U,
                   VCFG_COMPUTE_INTERFACE_SOCKETCAN, request);

    vcfg_write_request_t decoded;
    assert(vcfg_decode_write_request(request, &decoded) == VCFG_STATUS_OK);
    assert(decoded.sequence == 0x12345678U);
    assert(decoded.machine_id == 0x0001U);
    assert(decoded.protocol_id == 0x0101U);
    assert(decoded.compute_interface_id == VCFG_COMPUTE_INTERFACE_SOCKETCAN);

    uint8_t bad_crc[VCFG_FRAME_SIZE];
    memcpy(bad_crc, request, sizeof(bad_crc));
    bad_crc[12] ^= 1U;
    assert(vcfg_decode_write_request(bad_crc, &decoded)
           == VCFG_STATUS_BAD_FRAME);

    uint8_t unsupported[VCFG_FRAME_SIZE];
    memcpy(unsupported, request, sizeof(unsupported));
    unsupported[4] = 2U;
    uint32_t crc = vcfg_crc32(unsupported, 20U);
    unsupported[20] = (uint8_t)crc;
    unsupported[21] = (uint8_t)(crc >> 8U);
    unsupported[22] = (uint8_t)(crc >> 16U);
    unsupported[23] = (uint8_t)(crc >> 24U);
    assert(vcfg_decode_write_request(unsupported, &decoded)
           == VCFG_STATUS_UNSUPPORTED_VERSION);

    uint8_t unsupported_interface[VCFG_FRAME_SIZE];
    encode_request(2U, 0x0001U, 0x0101U, 3U, unsupported_interface);
    assert(vcfg_decode_write_request(unsupported_interface, &decoded)
           == VCFG_STATUS_UNSUPPORTED_COMPUTE_INTERFACE);

    uint8_t reserved_flags[VCFG_FRAME_SIZE];
    encode_request(3U, 0x0001U, 0x0101U,
                   VCFG_COMPUTE_INTERFACE_CAN, reserved_flags);
    reserved_flags[17] = 1U;
    crc = vcfg_crc32(reserved_flags, 20U);
    reserved_flags[20] = (uint8_t)crc;
    reserved_flags[21] = (uint8_t)(crc >> 8U);
    reserved_flags[22] = (uint8_t)(crc >> 16U);
    reserved_flags[23] = (uint8_t)(crc >> 24U);
    assert(vcfg_decode_write_request(reserved_flags, &decoded)
           == VCFG_STATUS_BAD_FRAME);

    vcfg_parser_t parser;
    vcfg_parser_init(&parser);
    uint8_t parsed[VCFG_FRAME_SIZE];
    assert(!vcfg_parser_feed(&parser, 0x00U, parsed));
    assert(!vcfg_parser_feed(&parser, 0x56U, parsed));
    assert(!vcfg_parser_feed(&parser, 0x56U, parsed));
    bool complete = false;
    for (size_t index = 1U; index < sizeof(request); index++) {
        complete = vcfg_parser_feed(&parser, request[index], parsed);
    }
    assert(complete);
    assert(memcmp(parsed, request, sizeof(request)) == 0);

    vcfg_write_ack_t ack = {
        .sequence = 0x12345678U,
        .status = VCFG_STATUS_OK,
        .active_slot = 1U,
        .generation = 2U,
        .machine_id = 0x0001U,
        .protocol_id = 0x0101U,
    };
    uint8_t ack_frame[VCFG_FRAME_SIZE];
    vcfg_encode_write_ack(&ack, ack_frame);
    assert(ack_frame[5] == VCFG_OPCODE_WRITE_SELECTION_ACK);
    assert(ack_frame[13] == 1U);
    assert(ack_frame[14] == 2U);
    assert(vcfg_crc32(ack_frame, 20U)
           == ((uint32_t)ack_frame[20] | ((uint32_t)ack_frame[21] << 8U)
               | ((uint32_t)ack_frame[22] << 16U)
               | ((uint32_t)ack_frame[23] << 24U)));
}

static void test_store(void)
{
    memory_flash_t flash;
    memset(&flash, 0xFF, sizeof(flash));
    flash.writes = 0U;
    flash.fail_write = false;
    flash.corrupt_after_write = false;
    vcu_store_backend_t backend = {
        .context = &flash,
        .read_slot = memory_read,
        .write_slot = memory_write,
    };

    vcu_selection_t selection;
    assert(!vcu_store_load(&backend, &selection));
    assert(vcu_store_save(&backend, 0x0001U, 0x0101U,
                          VCFG_COMPUTE_INTERFACE_SERIAL, &selection)
           == VCU_STORE_OK);
    assert(selection.active_slot == 0U);
    assert(selection.generation == 1U);
    assert(flash.writes == 1U);

    assert(selection.compute_interface_id == VCFG_COMPUTE_INTERFACE_SERIAL);

    assert(vcu_store_save(&backend, 0x0001U, 0x0101U,
                          VCFG_COMPUTE_INTERFACE_SERIAL, &selection)
           == VCU_STORE_OK);
    assert(selection.generation == 1U);
    assert(flash.writes == 1U);

    assert(vcu_store_save(&backend, 0x0001U, 0x0101U,
                          VCFG_COMPUTE_INTERFACE_SOCKETCAN, &selection)
           == VCU_STORE_OK);
    assert(selection.active_slot == 1U);
    assert(selection.generation == 2U);
    assert(flash.writes == 2U);
    assert(selection.compute_interface_id == VCFG_COMPUTE_INTERFACE_SOCKETCAN);

    flash.slots[1][12] ^= 1U;
    assert(vcu_store_load(&backend, &selection));
    assert(selection.active_slot == 0U);
    assert(selection.generation == 1U);

    flash.fail_write = true;
    assert(vcu_store_save(&backend, 0x0003U, 0x0103U,
                          VCFG_COMPUTE_INTERFACE_CAN, &selection)
           == VCU_STORE_WRITE_FAILED);
    flash.fail_write = false;
    flash.corrupt_after_write = true;
    assert(vcu_store_save(&backend, 0x0003U, 0x0103U,
                          VCFG_COMPUTE_INTERFACE_CAN, &selection)
           == VCU_STORE_VERIFY_FAILED);
}

int main(void)
{
    test_protocol();
    test_store();
    puts("VCFG protocol and A/B store tests passed");
    return 0;
}
