#ifndef AGRIBOT_HIL_ISOBUS_ENDPOINT_HPP
#define AGRIBOT_HIL_ISOBUS_ENDPOINT_HPP

#include "work_protocol.h"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_internal_control_function.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include <array>
#include <functional>

namespace agribot_hil {

constexpr std::uint32_t WorkCommandPGN = 0xef00;
constexpr std::uint32_t GatewayStatusPGN = 0xff60;
constexpr std::uint32_t WorkAckPGN = 0xff61;

struct EndpointHooks {
    std::function<bool(const isobus::CANMessageFrame &)> upstream;
    std::function<bool(std::uint16_t, const std::uint8_t *)> native;
    std::function<void(const char *, std::uint32_t, const std::uint8_t *, unsigned)> event;
};

class Endpoint {
public:
    Endpoint(bool gateway, EndpointHooks hooks, std::uint8_t preferred_address = 0xfe);
    void receive_upstream(const isobus::CANMessageFrame &frame);
    void receive_native(std::uint16_t id, const std::uint8_t bytes[8]);
    bool set_target(unsigned device, std::uint16_t value, bool enabled);
    void tick();
    bool network_ready() const;
    bool transmit(const isobus::CANMessageFrame &frame);
    void stop();
    static Endpoint *active;
private:
    static void receive_message(const isobus::CANMessage &message, void *parent);
    void handle_message(const isobus::CANMessage &message);
    bool gateway;
    EndpointHooks hooks;
    std::shared_ptr<isobus::InternalControlFunction> self;
    std::shared_ptr<isobus::PartneredControlFunction> partner;
    std::uint64_t partner_name;
    hil_work_gateway_t work;
    struct Target { std::uint16_t value = 0; std::uint32_t at = 0;
                    std::uint8_t sequence = 0; bool enabled = false, neutral_required = true; };
    std::array<Target, 3> targets;
    std::uint32_t last_send = 0, gateway_seen = 0;
    bool have_gateway = false, was_ready = false;
};
}
#endif
