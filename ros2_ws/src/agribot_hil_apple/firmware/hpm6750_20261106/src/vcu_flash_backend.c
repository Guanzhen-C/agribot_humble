#include "vcu_flash_backend.h"

#include <stdint.h>
#include <string.h>

#include "board.h"
#include "hpm_romapi.h"

typedef struct {
    xpi_nor_config_t config;
    uint32_t flash_size;
    uint32_t sector_size;
} flash_context_t;

static flash_context_t flash_context;

static uint32_t slot_offset(const flash_context_t *context, uint8_t slot)
{
    return context->flash_size - (2U - slot) * context->sector_size;
}

static bool read_slot(void *opaque, uint8_t slot, uint8_t *data, size_t length)
{
    flash_context_t *context = opaque;
    uint32_t aligned[VCU_SELECTION_RECORD_SIZE / sizeof(uint32_t)];

    if (slot > 1U || length != VCU_SELECTION_RECORD_SIZE) {
        return false;
    }
    hpm_stat_t status = rom_xpi_nor_read(
        BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &context->config,
        aligned, slot_offset(context, slot), VCU_SELECTION_RECORD_SIZE);
    if (status != status_success) {
        return false;
    }
    memcpy(data, aligned, length);
    return true;
}

static bool write_slot(void *opaque, uint8_t slot, const uint8_t *data,
                       size_t length)
{
    flash_context_t *context = opaque;
    uint32_t aligned[VCU_SELECTION_RECORD_SIZE / sizeof(uint32_t)];
    uint32_t offset;

    if (slot > 1U || length != VCU_SELECTION_RECORD_SIZE) {
        return false;
    }
    memcpy(aligned, data, length);
    offset = slot_offset(context, slot);

    hpm_stat_t status = rom_xpi_nor_erase_sector(
        BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &context->config,
        offset);
    if (status != status_success) {
        return false;
    }
    status = rom_xpi_nor_program(
        BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &context->config,
        aligned, offset, VCU_SELECTION_RECORD_SIZE);
    return status == status_success;
}

bool vcu_flash_backend_init(vcu_store_backend_t *backend)
{
    xpi_nor_config_option_t option = {0};
    option.header.U = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    option.option0.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    option.option1.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;

    hpm_stat_t status = rom_xpi_nor_auto_config(
        BOARD_APP_XPI_NOR_XPI_BASE, &flash_context.config, &option);
    if (status != status_success) {
        return false;
    }
    rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE,
                             &flash_context.config,
                             xpi_nor_property_total_size,
                             &flash_context.flash_size);
    rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE,
                             &flash_context.config,
                             xpi_nor_property_sector_size,
                             &flash_context.sector_size);
    if (flash_context.sector_size < VCU_SELECTION_RECORD_SIZE
        || flash_context.flash_size < flash_context.sector_size * 2U) {
        return false;
    }

    backend->context = &flash_context;
    backend->read_slot = read_slot;
    backend->write_slot = write_slot;
    return true;
}
