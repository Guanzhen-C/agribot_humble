#include "vcu_runtime_io.h"
#include "vcu_machine_profile.h"
#include "vcu_board_io.h"
#include "board.h"
#include "hpm_can_drv.h"
#include "hpm_uart_drv.h"
#include "hpm_mchtmr_drv.h"
#include <string.h>

#ifndef VCU_RUNTIME_ENABLE
#define VCU_RUNTIME_ENABLE 0
#endif

uint32_t vcu_runtime_now(void)
{
    return (uint32_t)(mchtmr_get_count(HPM_MCHTMR) / (clock_get_frequency(clock_mchtmr0) / 1000U));
}

#if VCU_RUNTIME_ENABLE
static bool ready;
static uint32_t last_tx;
static fgcp_can_parser_t input_can;
static uint8_t serial_input[FGCP_SERIAL_SIZE];
static size_t serial_length;
static uint32_t serial_at;

static bool init_can(CAN_Type *ptr, uint32_t bitrate)
{
    can_config_t config; can_get_default_config(&config);
    config.baudrate = bitrate; config.mode = can_mode_normal;
    /* The onboard CAN0 transceiver idles in standby until STB (PB14) is driven low. */
    if (ptr == HPM_CAN0) { board_init_can(ptr); board_can_transceiver_phy_set(ptr, true); }
    return can_init(ptr, &config, board_init_can_clock(ptr)) == status_success;
}
#endif

bool vcu_runtime_io_init(uint16_t machine_id, uint16_t protocol_id, uint8_t compute_interface)
{
#if VCU_RUNTIME_ENABLE
    uint32_t bitrate = vcu_machine_can_bitrate(machine_id);
    if (!bitrate) { return false; }
    vcu_board_route_compute_interface(compute_interface);
    ready = init_can(HPM_CAN0, bitrate);
    if (compute_interface != VCFG_COMPUTE_INTERFACE_SERIAL) {
        /* Native bridge keeps the machine bit rate end to end; FGCP uses 500k. */
        uint32_t compute_bitrate = vcu_select_mode(protocol_id, compute_interface) == VCU_MODE_BRIDGE
            ? vcu_bridge_profile(protocol_id)->bitrate : 500000U;
        ready = init_can(HPM_CAN1, compute_bitrate) && ready;
    } else {
        uart_config_t config; uart_default_config(HPM_UART13, &config);
        config.src_freq_in_hz = board_init_uart_clock(HPM_UART13); config.baudrate = 115200;
        ready = uart_init(HPM_UART13, &config) == status_success && ready;
    }
    serial_length = 0; input_can.has_drive = false;
    return ready;
#else
    (void)machine_id; (void)protocol_id; (void)compute_interface; return false;
#endif
}

#if VCU_RUNTIME_ENABLE
static bool classic_8(const can_receive_buf_t *rx)
{
    return !rx->extend_id && !rx->remote_frame && !rx->canfd_frame && rx->dlc == 8;
}

static bool send_std(CAN_Type *ptr, uint32_t id, const uint8_t data[8])
{
    can_transmit_buf_t tx; can_init_tx_message_std(&tx, (uint16_t)id, 8); memcpy(tx.data, data, 8);
    if (can_send_message_nonblocking(ptr, &tx) != status_success) { return false; }
    /* The SDK only requests one STB frame (TSONE); a 0x101..0x103 burst would
     * otherwise queue up until the STB is full. TSALL drains everything queued. */
    can_start_all_message_transmit(ptr);
    return true;
}

static void bridge_poll(vcu_bridge_t *bridge, uint32_t now)
{
    /* Forward only newly received whitelisted frames; nothing is held or replayed. */
    for (unsigned i = 0; i < 8 && can_is_data_available_in_receive_buffer(HPM_CAN1); ++i) {
        can_receive_buf_t rx; can_read_received_message(HPM_CAN1, &rx);
        if (vcu_bridge_from_compute(bridge, rx.id, rx.data, classic_8(&rx), now)) {
            vcu_bridge_sent(bridge, true, send_std(HPM_CAN0, rx.id, rx.data));
        }
    }
    for (unsigned i = 0; i < 8 && can_is_data_available_in_receive_buffer(HPM_CAN0); ++i) {
        can_receive_buf_t rx; can_read_received_message(HPM_CAN0, &rx);
        if (vcu_bridge_from_machine(bridge, rx.id, rx.data, classic_8(&rx), now)) {
            vcu_bridge_sent(bridge, false, send_std(HPM_CAN1, rx.id, rx.data));
        }
    }
}
#endif

void vcu_runtime_io_poll(vcu_runtime_t *state, vcu_bridge_t *bridge, vcu_mode_t mode,
                         uint8_t compute_interface, uint32_t now)
{
    vcu_runtime_tick(state, now);
#if VCU_RUNTIME_ENABLE
    if (!ready) { return; }
    if (mode == VCU_MODE_BRIDGE) { bridge_poll(bridge, now); return; }
    /* Bounded receive budget prevents a flooded bus from starving the safety tick. */
    for (unsigned i = 0; i < 8 && can_is_data_available_in_receive_buffer(HPM_CAN0); ++i) {
        can_receive_buf_t rx; can_read_received_message(HPM_CAN0, &rx);
        if (rx.extend_id || rx.remote_frame || rx.canfd_frame || rx.dlc != 8) { continue; }
        (void)vcu_runtime_feedback(state, (uint16_t)rx.id, rx.data, now);
    }
    if (compute_interface != VCFG_COMPUTE_INTERFACE_SERIAL) {
        for (unsigned i = 0; i < 8 && can_is_data_available_in_receive_buffer(HPM_CAN1); ++i) {
            can_receive_buf_t rx; can_read_received_message(HPM_CAN1, &rx);
            if (rx.id != 0x180 && rx.id != 0x181) { continue; }
            fgcp_command_t command;
            if (fgcp_decode_can(&input_can, (uint16_t)rx.id, rx.data, rx.dlc,
                    rx.extend_id || rx.canfd_frame, rx.remote_frame, now, &command)) {
                (void)vcu_runtime_accept(state, &command, now);
            } else if (!input_can.has_drive) { vcu_runtime_reject(state); }
            /* A valid stop/disable takes effect before waiting for its second frame. */
            if (rx.id == 0x180 && input_can.has_drive && (!(rx.data[4] & 1) || (rx.data[4] & 2))) {
                state->enabled = false;
            }
        }
    } else {
        if (serial_length && now - serial_at > 50U) { serial_length = 0; vcu_runtime_reject(state); }
        for (unsigned i = 0; i < 64; ++i) {
            uint8_t byte; if (uart_try_receive_byte(HPM_UART13, &byte) != status_success) { break; }
            if (!serial_length && byte != 'F') { continue; }
            serial_input[serial_length++] = byte; serial_at = now;
            if (serial_length == 2 && byte != 'G') { serial_length = byte == 'F' ? 1 : 0; continue; }
            if (serial_length != FGCP_SERIAL_SIZE) { continue; }
            fgcp_command_t command;
            if (fgcp_decode_serial(serial_input, serial_length, &command)) { (void)vcu_runtime_accept(state, &command, now); }
            else { vcu_runtime_reject(state); }
            serial_length = 0;
        }
    }
    if (now - last_tx >= 50U) {
        last_tx = now; uint16_t id; uint8_t data[8];
        if (vcu_runtime_output(state, now, &id, data)) {
            can_transmit_buf_t tx; can_init_tx_message_std(&tx, id, 8); memcpy(tx.data, data, 8);
            if (can_send_message_nonblocking(HPM_CAN0, &tx) != status_success) {
                state->fault |= VCU_FAULT_IO; state->enabled = false; state->primed = false;
            }
        }
    }
#else
    (void)bridge; (void)mode; (void)compute_interface;
#endif
}
