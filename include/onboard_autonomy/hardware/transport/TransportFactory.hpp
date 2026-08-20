#pragma once

#include "onboard_autonomy/mission/flight/Transport.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace onboard_autonomy::hardware::transport {

std::unique_ptr<mission::ports::Transport>
make_udp_transport(const std::string& bind_address, std::uint16_t port);

std::unique_ptr<mission::ports::Transport> make_serial_transport(
    const std::string& device,
    std::uint32_t baud_rate,
    std::chrono::milliseconds reconnect_interval = std::chrono::seconds(1));

} // namespace onboard_autonomy::hardware::transport
