#include "onboard_autonomy/hardware/mavlink/MavlinkEncoder.hpp"

#include <ardupilotmega/mavlink.h>

#include <algorithm>
#include <array>
#include <cmath>
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
constexpr float kCopterGuidedMode = 4.0F;
constexpr std::uint16_t kPositionOnlyTypeMask = 3576;

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

std::vector<std::uint8_t> encode_set_guided_mode(
    const std::uint8_t vehicle_system_id,
    const std::uint8_t confirmation,
    const std::uint8_t component_id) {
    return encode_command_long(vehicle_system_id,
        component_id,
        MAV_CMD_DO_SET_MODE,
        confirmation,
        {
            static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            kCopterGuidedMode,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
        });
}

std::vector<std::uint8_t> encode_arm(const std::uint8_t vehicle_system_id,
    const std::uint8_t confirmation,
    const std::uint8_t component_id) {
    return encode_command_long(vehicle_system_id,
        component_id,
        MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation,
        {
            1.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
        });
}

std::vector<std::uint8_t> encode_takeoff(const std::uint8_t vehicle_system_id,
    const double altitude_m,
    const std::uint8_t confirmation,
    const std::uint8_t component_id) {
    return encode_command_long(vehicle_system_id,
        component_id,
        MAV_CMD_NAV_TAKEOFF,
        confirmation,
        {
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            static_cast<float>(altitude_m),
        });
}

std::vector<std::uint8_t> encode_land(const std::uint8_t vehicle_system_id,
    const std::uint8_t confirmation,
    const std::uint8_t component_id) {
    return encode_command_long(vehicle_system_id,
        component_id,
        MAV_CMD_NAV_LAND,
        confirmation,
        {
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
        });
}

std::vector<std::uint8_t> encode_return_to_launch(
    const std::uint8_t vehicle_system_id,
    const std::uint8_t confirmation,
    const std::uint8_t component_id) {
    return encode_command_long(vehicle_system_id,
        component_id,
        MAV_CMD_NAV_RETURN_TO_LAUNCH,
        confirmation,
        {
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
        });
}

std::vector<std::uint8_t> encode_condition_yaw(
    const std::uint8_t vehicle_system_id,
    const double relative_yaw_degrees,
    const double yaw_speed_degrees_per_second,
    const std::uint8_t confirmation,
    const std::uint8_t component_id) {
    constexpr double kMaximumRelativeYawDegrees = 180.0;
    if (!std::isfinite(relative_yaw_degrees) ||
        std::abs(relative_yaw_degrees) > kMaximumRelativeYawDegrees ||
        !std::isfinite(yaw_speed_degrees_per_second) ||
        yaw_speed_degrees_per_second <= 0.0) {
        throw std::invalid_argument("invalid condition-yaw command");
    }

    constexpr float kClockwise = 1.0F;
    constexpr float kCounterClockwise = -1.0F;
    constexpr float kRelativeAngle = 1.0F;
    return encode_command_long(vehicle_system_id,
        component_id,
        MAV_CMD_CONDITION_YAW,
        confirmation,
        {
            static_cast<float>(std::abs(relative_yaw_degrees)),
            static_cast<float>(yaw_speed_degrees_per_second),
            relative_yaw_degrees >= 0.0 ? kClockwise : kCounterClockwise,
            kRelativeAngle,
            0.0F,
            0.0F,
            0.0F,
        });
}

std::vector<std::uint8_t> encode_local_position_target(
    const std::uint8_t vehicle_system_id,
    const double north_m,
    const double east_m,
    const double down_m,
    const std::uint8_t component_id) {
    mavlink_message_t message{};
    mavlink_msg_set_position_target_local_ned_pack(vehicle_system_id,
        component_id,
        &message,
        0,
        vehicle_system_id,
        0,
        MAV_FRAME_LOCAL_OFFSET_NED,
        kPositionOnlyTypeMask,
        static_cast<float>(north_m),
        static_cast<float>(east_m),
        static_cast<float>(down_m),
        0.0F,
        0.0F,
        0.0F,
        0.0F,
        0.0F,
        0.0F,
        0.0F,
        0.0F);

    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto length = mavlink_msg_to_send_buffer(buffer.data(), &message);
    return {buffer.begin(), buffer.begin() + length};
}

std::vector<std::uint8_t> encode_landing_target(
    const std::uint8_t vehicle_system_id,
    const std::uint64_t time_usec,
    const double forward_m,
    const double right_m,
    const double down_m,
    const std::uint8_t component_id) {
    mavlink_message_t message{};
    const std::array<float, 4> orientation{
        1.0F,
        0.0F,
        0.0F,
        0.0F,
    };
    const auto distance =
        std::sqrt(forward_m * forward_m + right_m * right_m + down_m * down_m);

    mavlink_msg_landing_target_pack(vehicle_system_id,
        component_id,
        &message,
        time_usec,
        0,
        MAV_FRAME_BODY_FRD,
        0.0F,
        0.0F,
        static_cast<float>(distance),
        0.0F,
        0.0F,
        static_cast<float>(forward_m),
        static_cast<float>(right_m),
        static_cast<float>(down_m),
        orientation.data(),
        LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        1);

    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto length = mavlink_msg_to_send_buffer(buffer.data(), &message);
    return {buffer.begin(), buffer.begin() + length};
}

} // namespace onboard_autonomy::hardware::mavlink
