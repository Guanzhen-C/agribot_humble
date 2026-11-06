#ifndef VCU_SELECTION_STORE_H
#define VCU_SELECTION_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VCU_SELECTION_RECORD_SIZE 24U

typedef struct {
    uint32_t generation;
    uint16_t machine_id;
    uint16_t protocol_id;
    uint8_t compute_interface_id;
    uint8_t active_slot;
} vcu_selection_t;

typedef bool (*vcu_slot_read_fn)(void *context, uint8_t slot,
                                 uint8_t *data, size_t length);
typedef bool (*vcu_slot_write_fn)(void *context, uint8_t slot,
                                  const uint8_t *data, size_t length);

typedef struct {
    void *context;
    vcu_slot_read_fn read_slot;
    vcu_slot_write_fn write_slot;
} vcu_store_backend_t;

typedef enum {
    VCU_STORE_OK = 0,
    VCU_STORE_WRITE_FAILED,
    VCU_STORE_VERIFY_FAILED,
} vcu_store_status_t;

bool vcu_store_load(const vcu_store_backend_t *backend,
                    vcu_selection_t *selection);
vcu_store_status_t vcu_store_save(const vcu_store_backend_t *backend,
                                  uint16_t machine_id, uint16_t protocol_id,
                                  uint8_t compute_interface_id,
                                  vcu_selection_t *selection);

#endif
