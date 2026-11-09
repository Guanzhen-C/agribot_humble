#include "endpoint.hpp"
#include "isobus/isobus/can_hardware_abstraction.hpp"
#include "isobus/utility/system_timing.hpp"
#include <cstring>
#include <utility>

namespace agribot_hil {
Endpoint *Endpoint::active = nullptr;

static std::uint32_t now_ms() { return isobus::SystemTiming::get_timestamp_ms(); }

static isobus::NAME bench_name(bool gateway)
{
    // Unregistered bench identities, not a production manufacturer allocation.
    isobus::NAME name(0);
    name.set_arbitrary_address_capable(true);
    name.set_industry_group(2);
    name.set_device_class(0);
    name.set_function_code(130);
    name.set_manufacturer_code(2047);
    name.set_identity_number(gateway ? 0xa62 : 0xa61);
    return name;
}

Endpoint::Endpoint(bool gateway_, EndpointHooks hooks_, std::uint8_t preferred_address)
    : gateway(gateway_), hooks(std::move(hooks_))
{
    active = this;
    hil_work_init(&work);
    auto &network = isobus::CANNetworkManager::CANNetwork;
    auto peer = bench_name(!gateway);
    partner_name = peer.get_full_name();
    using Parameter = isobus::NAME::NAMEParameters;
    partner = network.create_partnered_control_function(0, {
        isobus::NAMEFilter(Parameter::IdentityNumber, gateway ? 0xa61 : 0xa62),
        isobus::NAMEFilter(Parameter::ManufacturerCode, 2047),
        isobus::NAMEFilter(Parameter::IndustryGroup, 2),
        isobus::NAMEFilter(Parameter::FunctionCode, 130)});
    self = network.create_internal_control_function(bench_name(gateway), 0,
        preferred_address == 0xfe ? (gateway ? 0x91 : 0x90) : preferred_address);
    for (auto pgn : std::array<std::uint32_t, 7>{{WorkCommandPGN, WorkAckPGN, GatewayStatusPGN,
                     0xff50, 0xff52, 0xff53, 0xff54}}) {
        network.add_any_control_function_parameter_group_number_callback(pgn, receive_message, this);
    }
}

bool Endpoint::transmit(const isobus::CANMessageFrame &frame)
{
    bool sent = hooks.upstream && hooks.upstream(frame);
    if (sent) { isobus::on_transmit_can_message_frame_from_hardware(frame); }
    return sent;
}

void Endpoint::receive_upstream(const isobus::CANMessageFrame &frame)
{
    if (frame.dataLength > 8) { return; }
    if (frame.isExtendedFrame) {
        isobus::receive_can_message_frame_from_hardware(frame);
    } else if (gateway && frame.identifier == 0x514 && frame.dataLength == 8 &&
               hil_work_xor(frame.data, 7) == frame.data[7]) {
        // Legacy drive frames share the upstream wire; never hold or replay them.
        if (hooks.native) { hooks.native(0x514, frame.data); }
    }
}

void Endpoint::receive_native(std::uint16_t id, const std::uint8_t bytes[8])
{
    if (!gateway || !bytes || hil_work_xor(bytes, 7) != bytes[7]) { return; }
    if (id >= 0x532 && id <= 0x534) {
        isobus::CANMessageFrame frame{};
        frame.identifier = id; frame.dataLength = 8;
        std::memcpy(frame.data, bytes, 8);
        if (hooks.upstream) { hooks.upstream(frame); }
    }
    if (!hil_work_feedback(&work, id, bytes, now_ms())) { return; }
    std::uint32_t pgn = id == 0x5b0 ? 0xff50U : id >= 0x5b2 && id <= 0x5b4 ? 0xff52U + id - 0x5b2 : 0;
    if (pgn && self->get_address_valid()) {
        isobus::CANNetworkManager::CANNetwork.send_can_message(pgn, bytes, 8, self);
    }
}

void Endpoint::receive_message(const isobus::CANMessage &message, void *parent)
{
    static_cast<Endpoint *>(parent)->handle_message(message);
}

void Endpoint::handle_message(const isobus::CANMessage &message)
{
    auto source = message.get_source_control_function();
    if (!source || !source->get_address_valid() || source->get_NAME().get_full_name() != partner_name ||
        message.get_data_length() != 8) { return; }
    const auto &data = message.get_data();
    auto pgn = message.get_identifier().get_parameter_group_number();
    if (gateway && pgn == WorkCommandPGN && message.get_destination_control_function() == self) {
        auto result = hil_work_accept(&work, data.data(), now_ms());
        std::uint8_t ack[8] = {HIL_WORK_VERSION, data[3], data[2], result, data[4], data[5], 0, 0};
        ack[7] = hil_work_xor(ack, 7);
        isobus::CANNetworkManager::CANNetwork.send_can_message(WorkAckPGN, ack, 8, self);
        if (hooks.event) { hooks.event("command", pgn, ack, 8); }
    } else if (!gateway && pgn == GatewayStatusPGN && data[0] == HIL_WORK_VERSION &&
               hil_work_xor(data.data(), 7) == data[7]) {
        have_gateway = true; gateway_seen = now_ms();
        if (hooks.event) { hooks.event("gateway", pgn, data.data(), 8); }
    } else if (!gateway && pgn == WorkAckPGN && data[0] == HIL_WORK_VERSION &&
               hil_work_xor(data.data(), 7) == data[7]) {
        if (data[1] >= 1 && data[1] <= 3 && data[3] != HIL_WORK_OK) {
            auto &target = targets[data[1] - 1];
            target.value = 0; target.enabled = false; target.neutral_required = true;
        }
        if (hooks.event) { hooks.event("ack", pgn, data.data(), 8); }
    } else if (!gateway && (pgn == 0xff50 || (pgn >= 0xff52 && pgn <= 0xff54))) {
        std::uint16_t id = pgn == 0xff50 ? 0x5b0 : 0x5b2 + pgn - 0xff52;
        if (hooks.event) { hooks.event("feedback", id, data.data(), 8); }
    }
}

bool Endpoint::network_ready() const
{
    return self->get_address_valid() && partner->get_address_valid() &&
        (gateway || (have_gateway && now_ms() - gateway_seen < 500));
}

bool Endpoint::set_target(unsigned device, std::uint16_t value, bool enabled)
{
    if (gateway || device < 1 || device > 3 || value > (device == 3 ? 10000 : 8) || (!enabled && value)) {
        return false;
    }
    auto &target = targets[device - 1];
    if (value && (!network_ready() || target.neutral_required)) { return false; }
    target.value = value; target.enabled = enabled; target.at = now_ms();
    if (!value) { target.neutral_required = false; }
    return true;
}

void Endpoint::stop()
{
    hil_work_stop(&work);
    for (auto &target : targets) {
        target.value = 0; target.enabled = false; target.neutral_required = true;
    }
}

void Endpoint::tick()
{
    isobus::periodic_update_from_hardware();
    auto now = now_ms();
    bool ready = network_ready();
    if (was_ready && !ready) { stop(); }
    was_ready = ready;
    if (now - last_send < 100) { return; }
    last_send = now;
    if (gateway) {
        if (!ready) { hil_work_stop(&work); }
        std::uint8_t command[8]; hil_work_encode(&work, now, command);
        if (hooks.native) { hooks.native(0x580, command); }
        if (self->get_address_valid()) {
            std::uint8_t status[8] = {HIL_WORK_VERSION, static_cast<std::uint8_t>(ready), 0,
                self->get_address(), static_cast<std::uint8_t>(work.accepted),
                static_cast<std::uint8_t>(work.accepted >> 8), static_cast<std::uint8_t>(work.rejected), 0};
            for (unsigned i = 0; i < 3; ++i) {
                if (hil_work_ready(&work, i, now)) { status[2] |= 1U << i; }
            }
            status[7] = hil_work_xor(status, 7);
            isobus::CANNetworkManager::CANNetwork.send_can_message(GatewayStatusPGN, status, 8, self);
        }
    } else if (ready) {
        for (unsigned i = 0; i < 3; ++i) {
            auto &target = targets[i];
            if (now - target.at >= HIL_WORK_LEASE_MS) {
                target.value = 0; target.enabled = false; target.neutral_required = true;
            }
            std::uint8_t command[8];
            hil_work_command(i + 1, target.value, ++target.sequence, target.enabled, command);
            isobus::CANNetworkManager::CANNetwork.send_can_message(WorkCommandPGN, command, 8, self, partner);
        }
    }
}
}

namespace isobus {
bool send_can_message_frame_to_hardware(const CANMessageFrame &frame)
{
    return agribot_hil::Endpoint::active && agribot_hil::Endpoint::active->transmit(frame);
}
}
