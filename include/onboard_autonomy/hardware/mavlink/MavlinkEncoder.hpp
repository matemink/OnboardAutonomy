#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

namespace onboard_autonomy::hardware::mavlink {

inline constexpr std::uint8_t kCompanionComponentId{191};

std::vector<std::uint8_t> encode_companion_heartbeat(std::uint8_t system_id,
    std::uint8_t component_id = kCompanionComponentId);

std::vector<std::uint8_t> encode_set_message_interval(
    std::uint8_t vehicle_system_id,
    std::uint32_t message_id,
    std::uint32_t interval_microseconds,
    std::uint8_t confirmation = 0,
    std::uint8_t component_id = kCompanionComponentId);

std::vector<std::uint8_t> encode_battery_arming_voltage_request(
    std::uint8_t vehicle_system_id,
    std::uint8_t component_id = kCompanionComponentId);

std::vector<std::uint8_t> encode_parameter_request_read(
    std::uint8_t vehicle_system_id,
    std::string_view parameter_id,
    std::uint8_t component_id = kCompanionComponentId);

std::vector<std::uint8_t> encode_autopilot_version_request(
    std::uint8_t vehicle_system_id,
    std::uint8_t component_id = kCompanionComponentId);

} // namespace onboard_autonomy::hardware::mavlink
