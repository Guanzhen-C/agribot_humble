#include "endpoint.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
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
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

using nlohmann::json;
static volatile std::sig_atomic_t running = 1;
static void interrupted(int) { running = 0; }

static std::string hex(const std::uint8_t *bytes, unsigned length)
{
    std::ostringstream out;
    for (unsigned i = 0; i < length; ++i) {
        out << std::hex << std::setfill('0') << std::setw(2) << unsigned(bytes[i]);
    }
    return out.str();
}

int main(int argc, char **argv)
{
    if (argc != 3 || (std::string(argv[1]) != "client" && std::string(argv[1]) != "gateway")) {
        std::cerr << "Usage: hil_isobus_endpoint client|gateway VCAN_INTERFACE\n"; return 2;
    }
    std::string interface = argv[2];
    // The physical CDC owner bridges this explicitly HIL-only virtual interface.
    if (interface.rfind("vcan", 0) != 0 || interface.size() >= IFNAMSIZ) { return 2; }
    int bus = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    struct ifreq request{}; std::strncpy(request.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (bus < 0 || ioctl(bus, SIOCGIFINDEX, &request) < 0) { perror("SocketCAN"); return 2; }
    sockaddr_can address{}; address.can_family = AF_CAN; address.can_ifindex = request.ifr_ifindex;
    if (bind(bus, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) { perror("bind"); return 2; }
    agribot_hil::EndpointHooks hooks;
    hooks.upstream = [bus](const isobus::CANMessageFrame &frame) {
        can_frame wire{}; wire.can_id = frame.identifier | (frame.isExtendedFrame ? CAN_EFF_FLAG : 0);
        wire.can_dlc = frame.dataLength; std::memcpy(wire.data, frame.data, frame.dataLength);
        return write(bus, &wire, sizeof(wire)) == sizeof(wire);
    };
    hooks.native = [](std::uint16_t id, const std::uint8_t *data) {
        std::cout << json({{"kind", "native_tx"}, {"id", id}, {"data", hex(data, 8)}}).dump() << std::endl;
        return true;
    };
    hooks.event = [](const char *kind, std::uint32_t id, const std::uint8_t *data, unsigned length) {
        std::cout << json({{"kind", kind}, {"id", id}, {"data", hex(data, length)}}).dump() << std::endl;
    };
    agribot_hil::Endpoint endpoint(std::string(argv[1]) == "gateway", hooks);
    std::signal(SIGINT, interrupted); std::signal(SIGTERM, interrupted);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
    std::string input; bool last_ready = false;
    std::cout << json({{"kind", "started"}, {"role", argv[1]}, {"interface", interface},
                      {"stack_commit", "02f3fe910de5fdc73e2014394c2e80900490b9cc"}}).dump() << std::endl;
    while (running) {
        for (unsigned budget = 0; budget < 64; ++budget) {
            can_frame raw{}; auto count = read(bus, &raw, sizeof(raw));
            if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { break; }
            if (count != sizeof(raw) || raw.can_dlc > 8 || (raw.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG))) { break; }
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
                unsigned device, value, enable;
                bool ok = bool(line >> device >> value >> enable) && value <= 10000 && enable <= 1 &&
                          endpoint.set_target(device, value, enable);
                std::cout << json({{"kind", "target"}, {"accepted", ok}}).dump() << std::endl;
            } else if (command == "STOP") { endpoint.stop(); }
            else if (command == "NATIVE") {
                unsigned id; std::string data; line >> std::hex >> id >> data;
                if (id <= 0x7ff && data.size() == 16) {
                    std::uint8_t bytes[8]; bool valid = true;
                    for (unsigned i = 0; i < 8; ++i) {
                        try { bytes[i] = std::stoul(data.substr(i * 2, 2), nullptr, 16); }
                        catch (...) { valid = false; }
                    }
                    if (valid) { endpoint.receive_native(id, bytes); }
                }
            }
        }
        endpoint.tick();
        bool ready = endpoint.network_ready();
        if (ready != last_ready) {
            std::cout << json({{"kind", "network"}, {"ready", ready}}).dump() << std::endl;
            last_ready = ready;
        }
        poll(nullptr, 0, 2);
    }
    endpoint.stop(); endpoint.tick(); close(bus); return 0;
}
