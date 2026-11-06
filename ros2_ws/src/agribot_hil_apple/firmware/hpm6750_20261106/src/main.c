#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "hpm_uart_drv.h"
#include "vcfg_protocol.h"
#include "vcu_board_io.h"
#include "vcu_flash_backend.h"
#include "vcu_selection_store.h"
#include "vcu_machine_profile.h"
#include "vcu_runtime_io.h"
#include "vcu_telemetry.h"
#include "vcu_bridge.h"

#define SUPPORTED_MACHINE_ID VCU_MACHINE_TRACKED
#define SUPPORTED_PROTOCOL_ID VCU_PROTOCOL_IMPLEMENT

#ifndef VCU_CONFIG_SELF_TEST
#define VCU_CONFIG_SELF_TEST 0
#endif

/* With no valid stored selection the VCU serves the agribot RDK stack:
 * Ackermann steering machine, native CAN bridge on CAN1. */
#define DEFAULT_MACHINE_ID VCU_MACHINE_STEERING
#define DEFAULT_PROTOCOL_ID VCU_PROTOCOL_STEERING
#define DEFAULT_COMPUTE_INTERFACE VCFG_COMPUTE_INTERFACE_CAN
/* A selection write is applied only after compute input has been quiet this long. */
#define STANDBY_QUIET_MS 1000U

#if VCU_CONFIG_SELF_TEST
volatile uint32_t vcu_self_test_status;
#endif

typedef struct {
    uint16_t machine_id;
    uint16_t protocol_id;
    uint8_t compute_interface;
    vcu_mode_t mode;
    bool hardware_ready;
    vcu_runtime_t runtime;
    vcu_bridge_t bridge;
} vcu_active_t;

static void activate(vcu_active_t *active, uint16_t machine_id, uint16_t protocol_id,
                     uint8_t compute_interface)
{
    active->machine_id = machine_id;
    active->protocol_id = protocol_id;
    active->compute_interface = compute_interface;
    active->mode = vcu_select_mode(protocol_id, compute_interface);
    vcu_runtime_init(&active->runtime, machine_id,
        compute_interface == VCFG_COMPUTE_INTERFACE_SERIAL ? 16 : 8);
    (void)vcu_bridge_init(&active->bridge, active->mode == VCU_MODE_BRIDGE ? protocol_id : 0U);
    active->hardware_ready = vcu_runtime_io_init(machine_id, protocol_id, compute_interface);
}

static bool compute_quiet(const vcu_active_t *active, uint32_t now)
{
    if (active->mode == VCU_MODE_BRIDGE) {
        return !active->bridge.has_command || now - active->bridge.command_at >= STANDBY_QUIET_MS;
    }
    return !active->runtime.enabled
        && (!active->runtime.has_command || now - active->runtime.command_at >= STANDBY_QUIET_MS);
}

static void send_frame(const uint8_t *frame, size_t length)
{
    for (size_t index = 0; index < length; index++) {
        (void)uart_send_byte(BOARD_CONSOLE_UART_BASE, frame[index]);
    }
}

int main(void)
{
    board_init();

    vcu_store_backend_t store_backend;
    bool flash_ready = vcu_flash_backend_init(&store_backend);
    vcu_selection_t active_selection = {0};
    bool has_active_selection = flash_ready
        && vcu_store_load(&store_backend, &active_selection);
    if (has_active_selection) {
        vcfg_write_request_t stored = {
            .machine_id = active_selection.machine_id,
            .protocol_id = active_selection.protocol_id,
            .compute_interface_id = active_selection.compute_interface_id,
        };
        has_active_selection = vcu_validate_selection(&stored) == VCFG_STATUS_OK;
    }

#if VCU_CONFIG_SELF_TEST
    if (flash_ready) {
        vcu_store_status_t self_test_status = vcu_store_save(
            &store_backend, SUPPORTED_MACHINE_ID, SUPPORTED_PROTOCOL_ID,
            VCFG_COMPUTE_INTERFACE_SERIAL,
            &active_selection);
        has_active_selection = self_test_status == VCU_STORE_OK
            && vcu_store_load(&store_backend, &active_selection);
        vcu_self_test_status = has_active_selection
            ? VCFG_STATUS_OK : VCFG_STATUS_VERIFY_FAILED;
    } else {
        vcu_self_test_status = VCFG_STATUS_FLASH_WRITE_FAILED;
    }
#endif

    vcfg_parser_t parser;
    vcfg_parser_init(&parser);
    static vcu_active_t active;
    if (has_active_selection) {
        activate(&active, active_selection.machine_id, active_selection.protocol_id,
            active_selection.compute_interface_id);
    } else {
        activate(&active, DEFAULT_MACHINE_ID, DEFAULT_PROTOCOL_ID, DEFAULT_COMPUTE_INTERFACE);
    }
    vcu_board_route_compute_interface(active.compute_interface);

    while (true) {
        uint32_t now = vcu_runtime_now();
        vcu_runtime_io_poll(&active.runtime, &active.bridge, active.mode,
            active.compute_interface, now);
        uint8_t byte;
        if (uart_try_receive_byte(BOARD_CONSOLE_UART_BASE, &byte)
            != status_success) {
            continue;
        }
        uint8_t request_frame[VCFG_FRAME_SIZE];
        if (!vcfg_parser_feed(&parser, byte, request_frame)) {
            continue;
        }

        uint32_t query_sequence;
        if (vcu_decode_query(request_frame, VCU_QUERY_OPCODE, &query_sequence)) {
            uint8_t telemetry[VCU_TELEMETRY_SIZE];
            if (active.mode == VCU_MODE_BRIDGE) {
                vcu_encode_bridge_telemetry(&active.bridge, active.machine_id, active.protocol_id,
                    active.compute_interface, has_active_selection, active.hardware_ready,
                    now, query_sequence, telemetry);
            } else {
                vcu_encode_telemetry(&active.runtime, active.protocol_id, active.compute_interface,
                    has_active_selection, active.hardware_ready, now, query_sequence, telemetry);
            }
            send_frame(telemetry, sizeof(telemetry));
            continue;
        }
        if (vcu_decode_query(request_frame, VCU_IDENTITY_QUERY_OPCODE, &query_sequence)) {
            uint8_t identity[VCU_IDENTITY_SIZE];
            vcu_encode_identity(active.machine_id, active.protocol_id, active.hardware_ready,
                active.mode == VCU_MODE_BRIDGE, query_sequence, identity);
            send_frame(identity, sizeof(identity));
            continue;
        }

        vcfg_write_request_t request;
        vcfg_status_t status = vcfg_decode_write_request(request_frame,
                                                         &request);
        if (status == VCFG_STATUS_OK) {
            status = vcu_validate_selection(&request);
        }
        if (status == VCFG_STATUS_OK && active.hardware_ready && !compute_quiet(&active, now)) {
            /* Flash erase must never suspend a running safety scheduler: accept a
             * new selection only while no compute command is arriving. */
            status = VCFG_STATUS_VCU_NOT_STANDBY;
        }
        if (status == VCFG_STATUS_OK && !flash_ready) {
            status = VCFG_STATUS_FLASH_WRITE_FAILED;
        }
        if (status == VCFG_STATUS_OK) {
            vcu_store_status_t store_status = vcu_store_save(
                &store_backend, request.machine_id, request.protocol_id,
                request.compute_interface_id,
                &active_selection);
            if (store_status == VCU_STORE_WRITE_FAILED) {
                status = VCFG_STATUS_FLASH_WRITE_FAILED;
            } else if (store_status == VCU_STORE_VERIFY_FAILED) {
                status = VCFG_STATUS_VERIFY_FAILED;
            } else {
                has_active_selection = true;
            }
        }

        vcfg_write_ack_t ack = {
            .sequence = request.sequence,
            .status = status,
            .active_slot = has_active_selection
                ? active_selection.active_slot : 0U,
            .generation = has_active_selection
                ? (uint16_t)active_selection.generation : 0U,
            .machine_id = has_active_selection
                ? active_selection.machine_id : request.machine_id,
            .protocol_id = has_active_selection
                ? active_selection.protocol_id : request.protocol_id,
        };
        uint8_t ack_frame[VCFG_FRAME_SIZE];
        vcfg_encode_write_ack(&ack, ack_frame);
        send_frame(ack_frame, sizeof(ack_frame));
        if (status == VCFG_STATUS_OK) {
            /* Adapt to the newly stored protocol without a reboot. */
            activate(&active, active_selection.machine_id, active_selection.protocol_id,
                active_selection.compute_interface_id);
        }
    }
}
