#include "vcu_selection_store.h"

#include <string.h>

#include "vcfg_protocol.h"

#define VCU_RECORD_SCHEMA_VERSION 1U

static const uint8_t record_magic[4] = {0x56U, 0x43U, 0x55U, 0x53U};

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

static void encode_record(const vcu_selection_t *selection,
                          uint8_t record[VCU_SELECTION_RECORD_SIZE])
{
    memset(record, 0, VCU_SELECTION_RECORD_SIZE);
    memcpy(record, record_magic, sizeof(record_magic));
    write_u16_le(&record[4], VCU_RECORD_SCHEMA_VERSION);
    write_u16_le(&record[6], VCU_SELECTION_RECORD_SIZE);
    write_u32_le(&record[8], selection->generation);
    write_u16_le(&record[12], selection->machine_id);
    write_u16_le(&record[14], selection->protocol_id);
    record[16] = selection->compute_interface_id;
    write_u32_le(&record[20], vcfg_crc32(record, 20U));
}

static bool decode_record(const uint8_t record[VCU_SELECTION_RECORD_SIZE],
                          uint8_t slot, vcu_selection_t *selection)
{
    if (memcmp(record, record_magic, sizeof(record_magic)) != 0
        || read_u16_le(&record[4]) != VCU_RECORD_SCHEMA_VERSION
        || read_u16_le(&record[6]) != VCU_SELECTION_RECORD_SIZE
        || read_u32_le(&record[20]) != vcfg_crc32(record, 20U)) {
        return false;
    }

    selection->generation = read_u32_le(&record[8]);
    selection->machine_id = read_u16_le(&record[12]);
    selection->protocol_id = read_u16_le(&record[14]);
    selection->compute_interface_id = record[16];
    selection->active_slot = slot;
    return selection->generation != 0U
        && selection->compute_interface_id
            <= VCFG_COMPUTE_INTERFACE_SOCKETCAN;
}

static bool generation_is_newer(uint32_t candidate, uint32_t current)
{
    return (int32_t)(candidate - current) > 0;
}

bool vcu_store_load(const vcu_store_backend_t *backend,
                    vcu_selection_t *selection)
{
    uint8_t raw_a[VCU_SELECTION_RECORD_SIZE];
    uint8_t raw_b[VCU_SELECTION_RECORD_SIZE];
    vcu_selection_t selection_a;
    vcu_selection_t selection_b;
    bool valid_a = backend->read_slot(backend->context, 0U, raw_a,
                                      sizeof(raw_a))
        && decode_record(raw_a, 0U, &selection_a);
    bool valid_b = backend->read_slot(backend->context, 1U, raw_b,
                                      sizeof(raw_b))
        && decode_record(raw_b, 1U, &selection_b);

    if (!valid_a && !valid_b) {
        return false;
    }
    if (valid_b && (!valid_a
                    || generation_is_newer(selection_b.generation,
                                           selection_a.generation))) {
        *selection = selection_b;
    } else {
        *selection = selection_a;
    }
    return true;
}

vcu_store_status_t vcu_store_save(const vcu_store_backend_t *backend,
                                  uint16_t machine_id, uint16_t protocol_id,
                                  uint8_t compute_interface_id,
                                  vcu_selection_t *selection)
{
    vcu_selection_t current;
    bool has_current = vcu_store_load(backend, &current);

    if (has_current && current.machine_id == machine_id
        && current.protocol_id == protocol_id
        && current.compute_interface_id == compute_interface_id) {
        *selection = current;
        return VCU_STORE_OK;
    }

    vcu_selection_t next = {
        .generation = has_current ? current.generation + 1U : 1U,
        .machine_id = machine_id,
        .protocol_id = protocol_id,
        .compute_interface_id = compute_interface_id,
        .active_slot = has_current ? (uint8_t)(current.active_slot ^ 1U) : 0U,
    };
    if (next.generation == 0U) {
        next.generation = 1U;
    }

    uint8_t encoded[VCU_SELECTION_RECORD_SIZE];
    encode_record(&next, encoded);
    if (!backend->write_slot(backend->context, next.active_slot, encoded,
                             sizeof(encoded))) {
        return VCU_STORE_WRITE_FAILED;
    }

    uint8_t verified_raw[VCU_SELECTION_RECORD_SIZE];
    vcu_selection_t verified;
    if (!backend->read_slot(backend->context, next.active_slot, verified_raw,
                            sizeof(verified_raw))
        || !decode_record(verified_raw, next.active_slot, &verified)
        || verified.generation != next.generation
        || verified.machine_id != next.machine_id
        || verified.protocol_id != next.protocol_id
        || verified.compute_interface_id != next.compute_interface_id) {
        return VCU_STORE_VERIFY_FAILED;
    }

    *selection = verified;
    return VCU_STORE_OK;
}
