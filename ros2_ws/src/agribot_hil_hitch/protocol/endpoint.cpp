#include "endpoint.hpp"
#include "isobus/isobus/can_hardware_abstraction.hpp"
#include "isobus/utility/system_timing.hpp"
#include <cstring>
#include <cmath>
#include <utility>

namespace agribot_hitch {
Endpoint *Endpoint::active = nullptr;
static std::uint32_t now_ms() { return isobus::SystemTiming::get_timestamp_ms(); }
static isobus::NAME name(bool gateway)
{
    isobus::NAME result(0);
    result.set_arbitrary_address_capable(true);
    result.set_industry_group(2); result.set_device_class(0); result.set_function_code(130);
    result.set_manufacturer_code(2047); /* Unregistered, isolated bench only. */
    result.set_identity_number(gateway ? 0xb72 : 0xb71);
    return result;
}
Endpoint::Endpoint(bool gateway, Hooks hooks) : gateway_(gateway), hooks_(std::move(hooks))
{
    active = this;
    auto &network = isobus::CANNetworkManager::CANNetwork;
    using Parameter = isobus::NAME::NAMEParameters;
    partner_name_ = name(!gateway).get_full_name();
    partner_ = network.create_partnered_control_function(0, {
        isobus::NAMEFilter(Parameter::IdentityNumber, gateway ? 0xb71 : 0xb72),
        isobus::NAMEFilter(Parameter::ManufacturerCode, 2047),
        isobus::NAMEFilter(Parameter::IndustryGroup, 2),
        isobus::NAMEFilter(Parameter::FunctionCode, 130)});
    self_ = network.create_internal_control_function(name(gateway), 0, gateway ? 0x95 : 0x94);
    for (auto pgn : {CommandPGN, FeedbackPGN, StatusPGN, AckPGN, PowerSettingsPGN,
                    PowerEnginePGN, PowerTransmissionPGN, PowerVehiclePGN, PowerLoadPGN,
                    PowerStatusPGN, PowerAckPGN}) {
        network.add_any_control_function_parameter_group_number_callback(pgn, receive, this);
    }
    hh_init(&controller_); hh_make_command(HH_HOLD, 0, 0, 0, false, target_);
    pt_init(&powertrain_); pt_make_profile(PT_OFF, 0, 1600, 0, 0, power_target_);
    set_powertrain_settings(600, 150);
}
bool Endpoint::transmit(const isobus::CANMessageFrame &frame)
{
    bool sent = hooks_.upstream && hooks_.upstream(frame);
    if (sent) { isobus::on_transmit_can_message_frame_from_hardware(frame); }
    return sent;
}
void Endpoint::receive_upstream(const isobus::CANMessageFrame &frame)
{
    if (frame.dataLength > 8) { return; }
    if (frame.isExtendedFrame) { isobus::receive_can_message_frame_from_hardware(frame); return; }
    /* New tractor plant protocol ONLY; never forward 0x181/0x514/0x580. */
    const auto *data = frame.data;
    if (!gateway_ || frame.dataLength != 8 || frame.identifier != HH_DRIVE || !hh_valid(data) || data[5] != HH_VERSION ||
        data[4] > 1) { return; }
    int velocity = static_cast<std::int16_t>(data[0] | data[1] << 8);
    int steering = static_cast<std::int16_t>(data[2] | data[3] << 8);
    if (velocity < 0 || velocity > 500 || steering < -6100 || steering > 6100 ||
        (!data[4] && (velocity || steering))) { return; }
    auto now = now_ms();
    bool fresh = drive_primed_ && now - drive_at_ < 250;
    bool neutral = !data[4] && !velocity && !steering;
    if (!neutral && (!fresh || !ready() || !hh_next(drive_sequence_, data[6]))) { return; }
    if (fresh && !hh_next(drive_sequence_, data[6])) { return; }
    drive_primed_ = true; drive_sequence_ = data[6]; drive_at_ = now;
    pt_drive(&powertrain_, data, now);
    if (hooks_.native) { hooks_.native(HH_DRIVE, data); }
}
void Endpoint::receive_native(std::uint16_t id, const std::uint8_t data[8])
{
    if (!gateway_ || !hh_valid(data)) { return; }
    if (id >= PT_ENGINE_SENSOR && id <= PT_LOAD_SENSOR) {
        if (id <= PT_VEHICLE_SENSOR) { pt_sensor(&powertrain_, id, data, now_ms()); }
        if (self_->get_address_valid()) {
            isobus::CANNetworkManager::CANNetwork.send_can_message(PowerEnginePGN+id-PT_ENGINE_SENSOR, data, 8, self_);
        }
        return;
    }
    if (id == HH_DRIVE_STATE) {
        isobus::CANMessageFrame frame{}; frame.identifier = id; frame.dataLength = 8;
        std::memcpy(frame.data, data, 8);
        if (hooks_.upstream) { hooks_.upstream(frame); }
    }
    if (id != HH_SENSOR || !hh_sensor(&controller_, data, now_ms())) { return; }
    if (self_->get_address_valid()) {
        isobus::CANNetworkManager::CANNetwork.send_can_message(FeedbackPGN, data, 8, self_);
    }
}
void Endpoint::receive(const isobus::CANMessage &message, void *parent)
{ static_cast<Endpoint *>(parent)->message(message); }
void Endpoint::message(const isobus::CANMessage &message)
{
    auto source = message.get_source_control_function();
    if (!source || !source->get_address_valid() || source->get_NAME().get_full_name() != partner_name_ ||
        message.get_data_length() != 8 || !hh_valid(message.get_data().data())) { return; }
    auto pgn = message.get_identifier().get_parameter_group_number();
    const auto *data = message.get_data().data();
    if (gateway_ && pgn == CommandPGN && data[0] == PT_VERSION && message.get_destination_control_function() == self_) {
        auto result = pt_profile(&powertrain_, data, now_ms());
        std::uint8_t ack[8] = {PT_VERSION, result, data[1], data[2], 0, 0, data[6], 0};
        hh_seal(ack);
        isobus::CANNetworkManager::CANNetwork.send_can_message(PowerAckPGN, ack, 8, self_);
        if (hooks_.event) { hooks_.event("powertrain_ack", PowerAckPGN, ack, 8); }
    } else if (gateway_ && pgn == PowerSettingsPGN && data[0] == PT_VERSION) {
        auto result = pt_settings(&powertrain_, data, now_ms());
        std::uint8_t ack[8] = {PT_VERSION, result, 0, 0, 1, 0, data[6], 0}; hh_seal(ack);
        isobus::CANNetworkManager::CANNetwork.send_can_message(PowerAckPGN, ack, 8, self_);
    } else if (gateway_ && pgn == CommandPGN && data[0] == HH_VERSION && message.get_destination_control_function() == self_) {
        auto result = hh_command(&controller_, data, now_ms());
        std::uint8_t ack[8] = {HH_VERSION, result, data[1], data[5], data[2], data[3], 0, 0};
        hh_seal(ack);
        isobus::CANNetworkManager::CANNetwork.send_can_message(AckPGN, ack, 8, self_);
        if (hooks_.event) { hooks_.event("ack", AckPGN, ack, 8); }
    } else if (!gateway_ && pgn >= PowerEnginePGN && pgn <= PowerAckPGN) {
        const char *kinds[] = {"powertrain_engine", "powertrain_transmission", "powertrain_vehicle",
                              "powertrain_load", "powertrain_status", "powertrain_ack"};
        if (pgn == PowerAckPGN && data[1] != HH_OK) { stop_powertrain(); }
        if (hooks_.event) { hooks_.event(kinds[pgn-PowerEnginePGN], pgn, data, 8); }
    } else if (!gateway_ && pgn == StatusPGN && data[0] == HH_VERSION) {
        have_status_ = true; status_at_ = now_ms(); remote_sensor_ = data[2] != 0;
        if (hooks_.event) { hooks_.event("status", pgn, data, 8); }
    } else if (!gateway_ && pgn == FeedbackPGN) {
        if (hooks_.event) { hooks_.event("sensor", pgn, data, 8); }
    } else if (!gateway_ && pgn == AckPGN && data[0] == HH_VERSION) {
        if (data[1] != HH_OK) { stop(); }
        if (hooks_.event) { hooks_.event("ack", pgn, data, 8); }
    }
}
bool Endpoint::ready() const
{
    return self_->get_address_valid() && partner_->get_address_valid() &&
        (gateway_ || (have_status_ && remote_sensor_ && now_ms() - status_at_ < 300));
}
bool Endpoint::set_target(unsigned mode, unsigned position, unsigned speed, bool enable)
{
    if (gateway_ || mode > HH_LOWER || position > 10000 || speed > 100 ||
        (mode != HH_HOLD && (!speed || !enable || !ready() || require_hold_))) { return false; }
    bool neutral = mode == HH_HOLD && !enable;
    if (neutral) { require_hold_ = false; }
    hh_make_command(mode, position, speed, 0, enable, target_);
    target_at_ = now_ms(); have_target_ = true;
    return true;
}
void Endpoint::stop()
{
    hh_stop(&controller_); have_target_ = false; require_hold_ = true;
    hh_make_command(HH_HOLD, 0, 0, 0, false, target_);
}
bool Endpoint::set_powertrain(unsigned mode, unsigned gear, unsigned rpm, float ratio)
{
    if (gateway_ || mode > PT_CVT_MANUAL || gear > 4 || !std::isfinite(ratio) ||
        (mode && (rpm < 800 || rpm > 2200 || rpm%20)) ||
        (mode == PT_POWERSHIFT_MANUAL ? !gear : gear != 0) ||
        (mode == PT_CVT_MANUAL ? ratio < 200 || ratio > 900 : ratio != 0) ||
        (mode && (!ready() || power_require_hold_))) { return false; }
    if (!mode) { power_require_hold_ = false; }
    pt_make_profile(mode, gear, rpm, ratio, 0, power_target_);
    power_target_at_ = now_ms(); power_have_target_ = true; return true;
}
bool Endpoint::set_powertrain_settings(unsigned shift_ms, float rate)
{
    if (shift_ms < 200 || shift_ms > 2000 || shift_ms%10 || !std::isfinite(rate) || rate < 10 || rate > 300) { return false; }
    std::memset(power_settings_, 0, 8); power_settings_[0] = PT_VERSION; power_settings_[1] = shift_ms/10;
    unsigned wire = static_cast<unsigned>(rate*10+.5f); power_settings_[2] = wire&255; power_settings_[3] = wire>>8;
    hh_seal(power_settings_); return true;
}
void Endpoint::stop_powertrain()
{
    power_have_target_ = false; power_require_hold_ = true;
    pt_make_profile(PT_OFF, 0, 1600, 0, 0, power_target_);
}
void Endpoint::tick()
{
    isobus::periodic_update_from_hardware();
    auto now = now_ms(); bool network = ready();
    if (last_ready_ && !network) { stop(); stop_powertrain(); }
    last_ready_ = network;
    if (gateway_ && now - control_at_ >= 20) {
        control_at_ = now; hh_step(&controller_, now, network);
        std::uint8_t command[8]; hh_output(&controller_, command);
        if (hooks_.native) { hooks_.native(HH_VALVE, command); }
        pt_step(&powertrain_, now, network);
        std::uint8_t engine[8], transmission[8]; pt_output(&powertrain_, engine, transmission);
        if (hooks_.native) { hooks_.native(PT_ENGINE, engine); hooks_.native(PT_TRANSMISSION, transmission); }
    }
    if (now - publish_at_ < 100) { return; }
    publish_at_ = now;
    if (gateway_) {
        std::uint8_t status[8] = {HH_VERSION, static_cast<std::uint8_t>(network),
            static_cast<std::uint8_t>(controller_.have_sensor && !controller_.sensor_fault &&
                now - controller_.sensor_at < HH_SENSOR_TIMEOUT), controller_.fault,
            self_->get_address(), 0, 0, 0};
        hh_seal(status);
        if (self_->get_address_valid()) {
            isobus::CANNetworkManager::CANNetwork.send_can_message(StatusPGN, status, 8, self_);
            std::uint8_t power_status[8]; pt_status(&powertrain_, power_status);
            isobus::CANNetworkManager::CANNetwork.send_can_message(PowerStatusPGN, power_status, 8, self_);
        }
    } else if (network) {
        if (!have_target_ || now - target_at_ >= 300) { stop(); }
        target_[5] = ++sequence_; hh_seal(target_);
        isobus::CANNetworkManager::CANNetwork.send_can_message(CommandPGN, target_, 8, self_, partner_);
        if (!power_have_target_ || now-power_target_at_ >= 300) { stop_powertrain(); }
        power_target_[6] = ++power_sequence_; hh_seal(power_target_);
        power_settings_[6] = ++config_sequence_; hh_seal(power_settings_);
        isobus::CANNetworkManager::CANNetwork.send_can_message(PowerSettingsPGN, power_settings_, 8, self_);
        isobus::CANNetworkManager::CANNetwork.send_can_message(CommandPGN, power_target_, 8, self_, partner_);
    }
}
}
namespace isobus {
bool send_can_message_frame_to_hardware(const CANMessageFrame &frame)
{ return agribot_hitch::Endpoint::active && agribot_hitch::Endpoint::active->transmit(frame); }
}
