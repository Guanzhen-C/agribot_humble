#include "endpoint.hpp"
#include <nlohmann/json.hpp>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
using nlohmann::json;
static volatile std::sig_atomic_t running = 1;
static void stop_signal(int) { running = 0; }
static std::string hex(const std::uint8_t *data, unsigned size)
{
    std::ostringstream out;
    for (unsigned i = 0; i < size; ++i) {
        out << std::hex << std::setfill('0') << std::setw(2) << unsigned(data[i]);
    }
    return out.str();
}
int main(int argc, char **argv)
{
    if (argc != 3 || (std::string(argv[1]) != "client" && std::string(argv[1]) != "gateway") ||
        std::string(argv[2]).find("vcan") != 0 || std::strlen(argv[2]) >= IFNAMSIZ) { return 2; }
    int bus = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    ifreq request{}; std::strncpy(request.ifr_name, argv[2], IFNAMSIZ - 1);
    if (bus < 0 || ioctl(bus, SIOCGIFINDEX, &request) < 0) { perror("SocketCAN"); return 2; }
    sockaddr_can address{}; address.can_family = AF_CAN; address.can_ifindex = request.ifr_ifindex;
    if (bind(bus, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) { return 2; }
    agribot_hitch::Hooks hooks;
    hooks.upstream = [bus](const isobus::CANMessageFrame &frame) {
        can_frame wire{}; wire.can_id = frame.identifier | (frame.isExtendedFrame ? CAN_EFF_FLAG : 0);
        wire.can_dlc = frame.dataLength; std::memcpy(wire.data, frame.data, frame.dataLength);
        return write(bus, &wire, sizeof(wire)) == sizeof(wire);
    };
    hooks.native = [](std::uint16_t id, const std::uint8_t *data) {
        std::cout << json({{"kind", "native"}, {"id", id}, {"data", hex(data, 8)}}).dump() << std::endl;
        return true;
    };
    hooks.event = [](const char *kind, std::uint32_t id, const std::uint8_t *data, unsigned size) {
        std::cout << json({{"kind", kind}, {"id", id}, {"data", hex(data, size)}}).dump() << std::endl;
    };
    agribot_hitch::Endpoint endpoint(std::string(argv[1]) == "gateway", hooks);
    std::signal(SIGINT, stop_signal); std::signal(SIGTERM, stop_signal);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
    std::string input; bool last_ready = false;
    std::cout << json({{"kind", "started"}, {"role", argv[1]}, {"interface", argv[2]},
        {"protocol", "hitch-hil-0x21"}}).dump() << std::endl;
    while (running) {
        for (unsigned budget = 0; budget < 64; ++budget) {
            can_frame raw{}; auto count = read(bus, &raw, sizeof(raw));
            if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { break; }
            if (count != sizeof(raw) || raw.can_dlc > 8 || raw.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) { break; }
            isobus::CANMessageFrame frame{};
            frame.identifier = raw.can_id & CAN_EFF_MASK; frame.isExtendedFrame = raw.can_id & CAN_EFF_FLAG;
            frame.dataLength = raw.can_dlc; std::memcpy(frame.data, raw.data, raw.can_dlc);
            endpoint.receive_upstream(frame);
        }
        char buffer[1024]; auto count = read(STDIN_FILENO, buffer, sizeof(buffer));
        if (count == 0) { break; }
        if (count > 0) { input.append(buffer, count); }
        if (input.size() > 16384) { endpoint.stop(); break; }
        for (auto end = input.find('\n'); end != std::string::npos; end = input.find('\n')) {
            std::istringstream line(input.substr(0, end)); input.erase(0, end + 1);
            std::string command; line >> command;
            if (command == "SET") {
                unsigned mode = 0, position = 0, speed = 0, enable = 0;
                bool accepted = bool(line >> mode >> position >> speed >> enable) && enable <= 1 &&
                    endpoint.set_target(mode, position, speed, enable);
                std::cout << json({{"kind", "target"}, {"accepted", accepted}}).dump() << std::endl;
            } else if (command == "POWER") {
                unsigned mode = 0, gear = 0, rpm = 0; float ratio = 0;
                bool accepted = bool(line >> mode >> gear >> rpm >> ratio) && endpoint.set_powertrain(mode, gear, rpm, ratio);
                std::cout << json({{"kind", "powertrain_target"}, {"accepted", accepted}}).dump() << std::endl;
            } else if (command == "SETTINGS") {
                unsigned milliseconds = 0; float rate = 0;
                bool accepted = bool(line >> milliseconds >> rate) && endpoint.set_powertrain_settings(milliseconds, rate);
                std::cout << json({{"kind", "powertrain_settings"}, {"accepted", accepted}}).dump() << std::endl;
            } else if (command == "STOP_POWERTRAIN") { endpoint.stop_powertrain(); }
            else if (command == "STOP") { endpoint.stop(); }
            else if (command == "NATIVE") {
                unsigned id = 0; std::string data; line >> std::hex >> id >> data;
                if (data.size() == 16 && (id == HH_SENSOR || id == HH_DRIVE_STATE || (id >= PT_ENGINE_SENSOR && id <= PT_LOAD_SENSOR))) {
                    std::uint8_t bytes[8]; bool valid = true;
                    for (unsigned i = 0; i < 8; ++i) {
                        try { bytes[i] = std::stoul(data.substr(i * 2, 2), nullptr, 16); }
                        catch (...) { valid = false; }
                    }
                    if (valid) { endpoint.receive_native(id, bytes); }
                }
            }
        }
        endpoint.tick(); bool ready = endpoint.ready();
        if (ready != last_ready) {
            std::cout << json({{"kind", "network"}, {"ready", ready}}).dump() << std::endl;
            last_ready = ready;
        }
        poll(nullptr, 0, 2);
    }
    endpoint.stop(); endpoint.stop_powertrain(); close(bus); return 0;
}
