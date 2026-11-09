#include "endpoint.hpp"
#include "isobus/utility/chrono_time_source.hpp"
extern "C" {
#include "board.h"
#include "hpm_can_drv.h"
#include "hpm_mchtmr_drv.h"
}
#include <cstring>

static std::uint64_t microseconds()
{
    return mchtmr_get_count(HPM_MCHTMR) /
        (clock_get_frequency(clock_mchtmr0) / 1000000U);
}

namespace isobus {
std::uint32_t ChronoTimeSource::get_current_time_ms() const { return microseconds() / 1000U; }
std::uint64_t ChronoTimeSource::get_current_time_us() const { return microseconds(); }
}

static bool send(CAN_Type *bus, std::uint32_t id, bool extended,
                 const std::uint8_t *bytes, std::uint8_t length)
{
    can_transmit_buf_t frame;
    if (extended) { can_init_tx_message_ext(&frame, id, length); }
    else { can_init_tx_message_std(&frame, id, length); }
    std::memcpy(frame.data, bytes, length);
    const auto status = can_send_message_nonblocking(bus, &frame);
    // SDK TSONE only drains one queued frame. After a no-ACK/restart interval,
    // a full STB otherwise rejects every new send without restarting its drain.
    if (status == status_success || status == status_can_tx_fifo_full) {
        can_start_all_message_transmit(bus);
    }
    return status == status_success;
}

static bool initialize(CAN_Type *bus)
{
    can_config_t config; can_get_default_config(&config);
    config.baudrate = 250000; config.mode = can_mode_normal;
    return can_init(bus, &config, board_init_can_clock(bus)) == status_success;
}

int main(void)
{
    board_init();
    board_init_can(HPM_CAN0);
    HPM_IOC->PAD[IOC_PAD_PE31].FUNC_CTL = IOC_PE31_FUNC_CTL_CAN1_TXD;
    HPM_IOC->PAD[IOC_PAD_PE30].FUNC_CTL = IOC_PE30_FUNC_CTL_CAN1_RXD;
    if (!initialize(HPM_CAN0) || !initialize(HPM_CAN1)) {
        printf("HIL ISOBUS CAN initialization failed\n");
        while (true) { board_delay_ms(100); }
    }
    printf("AGRIBOT HIL tracked ISOBUS 1.0; CAN1 mixed drive/ISOBUS; CAN0 native; 250k\n");
    agribot_hil::EndpointHooks hooks;
    hooks.upstream = [](const isobus::CANMessageFrame &frame) {
        return send(HPM_CAN1, frame.identifier, frame.isExtendedFrame, frame.data, frame.dataLength);
    };
    hooks.native = [](std::uint16_t id, const std::uint8_t *data) {
        return send(HPM_CAN0, id, false, data, 8);
    };
    agribot_hil::Endpoint endpoint(true, hooks);
    while (true) {
        for (unsigned side = 0; side < 2; ++side) {
            CAN_Type *bus = side ? HPM_CAN1 : HPM_CAN0;
            for (unsigned budget = 0; budget < 16 && can_is_data_available_in_receive_buffer(bus); ++budget) {
                can_receive_buf_t wire; can_read_received_message(bus, &wire);
                if (wire.remote_frame || wire.canfd_frame || wire.dlc > 8) { continue; }
                if (side) {
                    isobus::CANMessageFrame frame{};
                    frame.identifier = wire.id; frame.isExtendedFrame = wire.extend_id;
                    frame.dataLength = wire.dlc; frame.timestamp_us = microseconds();
                    std::memcpy(frame.data, wire.data, wire.dlc);
                    endpoint.receive_upstream(frame);
                } else if (!wire.extend_id && wire.id <= 0x7ff && wire.dlc == 8) {
                    endpoint.receive_native(static_cast<std::uint16_t>(wire.id), wire.data);
                }
            }
        }
        endpoint.tick();
        board_delay_ms(1);
    }
}
