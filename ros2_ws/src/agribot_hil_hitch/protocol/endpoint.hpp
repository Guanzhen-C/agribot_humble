#ifndef AGRIBOT_HIL_HITCH_ENDPOINT_HPP
#define AGRIBOT_HIL_HITCH_ENDPOINT_HPP
#include "hitch_control.h"
#include "powertrain.h"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include <functional>

namespace agribot_hitch {
struct Hooks {
    std::function<bool(const isobus::CANMessageFrame &)> upstream;
    std::function<bool(std::uint16_t, const std::uint8_t *)> native;
    std::function<void(const char *, std::uint32_t, const std::uint8_t *, unsigned)> event;
};
class Endpoint {
public:
    static Endpoint *active;
    static constexpr std::uint32_t CommandPGN = 0xef00, FeedbackPGN = 0xff70,
        StatusPGN = 0xff71, AckPGN = 0xff72;
    static constexpr std::uint32_t PowerSettingsPGN = 0xff73, PowerEnginePGN = 0xff74,
        PowerTransmissionPGN = 0xff75, PowerVehiclePGN = 0xff76, PowerLoadPGN = 0xff77,
        PowerStatusPGN = 0xff78, PowerAckPGN = 0xff79;
    Endpoint(bool gateway, Hooks hooks);
    void receive_upstream(const isobus::CANMessageFrame &frame);
    void receive_native(std::uint16_t id, const std::uint8_t data[8]);
    bool set_target(unsigned mode, unsigned position, unsigned speed, bool enable);
    bool set_powertrain(unsigned mode, unsigned gear, unsigned rpm, float ratio);
    bool set_powertrain_settings(unsigned shift_ms, float ratio_rate);
    void stop_powertrain();
    void stop();
    void tick();
    bool ready() const;
    bool transmit(const isobus::CANMessageFrame &frame);
private:
    static void receive(const isobus::CANMessage &message, void *parent);
    void message(const isobus::CANMessage &message);
    bool gateway_;
    Hooks hooks_;
    hh_controller controller_{};
    pt_controller powertrain_{};
    std::shared_ptr<isobus::InternalControlFunction> self_;
    std::shared_ptr<isobus::PartneredControlFunction> partner_;
    std::uint64_t partner_name_{};
    std::uint8_t target_[8]{}, sequence_{}, drive_sequence_{};
    std::uint32_t target_at_{}, status_at_{}, control_at_{}, publish_at_{}, drive_at_{};
    bool have_target_{}, have_status_{}, remote_sensor_{}, last_ready_{}, require_hold_{true};
    bool drive_primed_{};
    std::uint8_t power_target_[8]{}, power_settings_[8]{}, power_sequence_{}, config_sequence_{};
    std::uint32_t power_target_at_{};
    bool power_have_target_{}, power_require_hold_{true};
};
}
#endif
