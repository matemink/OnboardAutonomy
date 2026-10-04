#include "onboard_autonomy/hardware/mavlink/MavlinkEncoder.hpp"

#include <ardupilotmega/mavlink.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace onboard_autonomy::hardware::mavlink {
namespace {

constexpr std::size_t kCommandParameterCount = 7;
constexpr std::size_t kCommandParam1Index = 0;
constexpr std::size_t kCommandParam2Index = 1;
constexpr std::size_t kCommandParam3Index = 2;
constexpr std::size_t kCommandParam4Index = 3;
constexpr std::size_t kCommandParam5Index = 4;
constexpr std::size_t kCommandParam6Index = 5;
constexpr std::size_t kCommandParam7Index = 6;
constexpr std::size_t kMaximumParameterIdLength = 16;

using CommandParameters = std::array<float, kCommandParameterCount>;

std::vector<std::uint8_t> encode_command_long(
    const std::uint8_t vehicle_system_id,
    const std::uint8_t component_id,
    const std::uint16_t command,
    const std::uint8_t confirmation,
    const CommandParameters& parameters) {
    mavlink_message_t message{};
    mavlink_msg_command_long_pack(vehicle_system_id,
        component_id,
        &message,
        vehicle_system_id,
        0,
        command,
        confirmation,
        parameters[kCommandParam1Index],
        parameters[kCommandParam2Index],
        parameters[kCommandParam3Index],
        parameters[kCommandParam4Index],
        parameters[kCommandParam5Index],
        parameters[kCommandParam6Index],
        parameters[kCommandParam7Index]);

    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto length = mavlink_msg_to_send_buffer(buffer.data(), &message);
    return {buffer.begin(), buffer.begin() + length};
}

} // namespace

std::vector<std::uint8_t> encode_companion_heartbeat(
    const std::uint8_t system_id,
    const std::uint8_t component_id) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(system_id,
        component_id,
        &message,
        static_cast<std::uint8_t>(MAV_TYPE_ONBOARD_CONTROLLER),
        static_cast<std::uint8_t>(MAV_AUTOPILOT_INVALID),
        0,
        0,
        static_cast<std::uint8_t>(MAV_STATE_ACTIVE));

    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto length = mavlink_msg_to_send_buffer(buffer.data(), &message);
    return {buffer.begin(), buffer.begin() + length};
}

std::vector<std::uint8_t> encode_set_message_interval(
    const std::uint8_t vehicle_system_id,
    const std::uint32_t message_id,
    const std::uint32_t interval_microseconds,
    const std::uint8_t confirmation,
    const std::uint8_t component_id) {
    return encode_command_long(vehicle_system_id,
        component_id,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        confirmation,
        {
            static_cast<float>(message_id),
            static_cast<float>(interval_microseconds),
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
        });
}

std::vector<std::uint8_t> encode_battery_arming_voltage_request(
    const std::uint8_t vehicle_system_id,
    const std::uint8_t component_id) {
    return encode_parameter_request_read(vehicle_system_id,
        "BATT_ARM_VOLT",
        component_id);
}

std::vector<std::uint8_t> encode_parameter_request_read(
    const std::uint8_t vehicle_system_id,
    const std::string_view parameter_id,
    const std::uint8_t component_id) {
    if (parameter_id.empty() ||
        parameter_id.size() > kMaximumParameterIdLength) {
        throw std::invalid_argument(
            "MAVLink parameter id must contain 1 to 16 characters");
    }

    std::array<char, kMaximumParameterIdLength> encoded_parameter_id{};
    std::copy(parameter_id.begin(),
        parameter_id.end(),
        encoded_parameter_id.begin());

    mavlink_message_t message{};
    mavlink_msg_param_request_read_pack(vehicle_system_id,
        component_id,
        &message,
        vehicle_system_id,
        MAV_COMP_ID_AUTOPILOT1,
        encoded_parameter_id.data(),
        -1);

    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto length = mavlink_msg_to_send_buffer(buffer.data(), &message);
    return {buffer.begin(), buffer.begin() + length};
}

std::vector<std::uint8_t> encode_autopilot_version_request(
    const std::uint8_t vehicle_system_id,
    const std::uint8_t component_id) {
    return encode_command_long(vehicle_system_id,
        component_id,
        MAV_CMD_REQUEST_MESSAGE,
        0,
        {
            static_cast<float>(MAVLINK_MSG_ID_AUTOPILOT_VERSION),
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            1.0F,
        });
}

} // namespace onboard_autonomy::hardware::mavlink
