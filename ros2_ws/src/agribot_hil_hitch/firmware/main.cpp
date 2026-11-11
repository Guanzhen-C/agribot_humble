#include "endpoint.hpp"
#include "isobus/utility/chrono_time_source.hpp"
extern "C" {
#include "board.h"
#include "hpm_can_drv.h"
#include "hpm_mchtmr_drv.h"
}
#include <cstring>
static std::uint64_t micros()
{ return mchtmr_get_count(HPM_MCHTMR) / (clock_get_frequency(clock_mchtmr0) / 1000000U); }
namespace isobus {
std::uint32_t ChronoTimeSource::get_current_time_ms() const { return micros() / 1000U; }
std::uint64_t ChronoTimeSource::get_current_time_us() const { return micros(); }
}
static bool send(CAN_Type *bus, std::uint32_t id, bool extended, const std::uint8_t *data, unsigned size)
{
    can_transmit_buf_t frame;
    if (extended) { can_init_tx_message_ext(&frame, id, size); }
    else { can_init_tx_message_std(&frame, id, size); }
    std::memcpy(frame.data, data, size);
    auto result = can_send_message_nonblocking(bus, &frame);
    if (result == status_success || result == status_can_tx_fifo_full) { can_start_all_message_transmit(bus); }
    return result == status_success;
}
static bool initialize(CAN_Type *bus)
{
    can_config_t config; can_get_default_config(&config);
    config.baudrate = 250000; config.mode = can_mode_normal;
    return can_init(bus, &config, board_init_can_clock(bus)) == status_success;
}
int main(void)
{
    board_init(); board_init_can(HPM_CAN0);
    HPM_IOC->PAD[IOC_PAD_PE31].FUNC_CTL = IOC_PE31_FUNC_CTL_CAN1_TXD;
    HPM_IOC->PAD[IOC_PAD_PE30].FUNC_CTL = IOC_PE30_FUNC_CTL_CAN1_RXD;
    if (!initialize(HPM_CAN0) || !initialize(HPM_CAN1)) {
        printf("HITCH HIL CAN init failed\n");
        while (true) { board_delay_ms(100); }
    }
    printf("AGRIBOT HIL hitch 0x21 + powertrain 0x22; 6750 loops; CAN1 ISOBUS; CAN0 plant; 250k\n");
    agribot_hitch::Hooks hooks;
    hooks.upstream = [](const isobus::CANMessageFrame &frame) {
        return send(HPM_CAN1, frame.identifier, frame.isExtendedFrame, frame.data, frame.dataLength);
    };
    hooks.native = [](std::uint16_t id, const std::uint8_t *data) {
        return send(HPM_CAN0, id, false, data, 8);
    };
    agribot_hitch::Endpoint endpoint(true, hooks);
    while (true) {
        for (unsigned side = 0; side < 2; ++side) {
            auto bus = side ? HPM_CAN1 : HPM_CAN0;
            for (unsigned budget = 0; budget < 32 && can_is_data_available_in_receive_buffer(bus); ++budget) {
                can_receive_buf_t raw; can_read_received_message(bus, &raw);
                if (raw.remote_frame || raw.canfd_frame || raw.dlc > 8) { continue; }
                if (side) {
                    isobus::CANMessageFrame frame{};
                    frame.identifier = raw.id; frame.isExtendedFrame = raw.extend_id;
                    frame.dataLength = raw.dlc; frame.timestamp_us = micros();
                    std::memcpy(frame.data, raw.data, raw.dlc); endpoint.receive_upstream(frame);
                } else if (!raw.extend_id && raw.dlc == 8) {
                    endpoint.receive_native(raw.id, raw.data);
                }
            }
        }
        endpoint.tick(); board_delay_ms(1);
    }
}
