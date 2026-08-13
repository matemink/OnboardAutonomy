#define MAVLINK_USE_MESSAGE_INFO
#include "onboard_autonomy/adapters/mavlink/MavlinkDecoder.hpp"

#include <ardupilotmega/mavlink.h>

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace onboard_autonomy::adapters::mavlink {
namespace {

std::optional<double> battery_voltage(const mavlink_battery_status_t& battery) {
    double total_voltage = 0.0;
    bool has_voltage = false;

    const auto add_voltage = [&total_voltage, &has_voltage](
                                 const std::uint16_t millivolts) {
        if (millivolts > 0 &&
            millivolts != std::numeric_limits<std::uint16_t>::max()) {
            total_voltage += static_cast<double>(millivolts) / 1000.0;
            has_voltage = true;
        }
    };

    for (const auto millivolts : battery.voltages) {
        add_voltage(millivolts);
    }
    for (const auto millivolts : battery.voltages_ext) {
        add_voltage(millivolts);
    }

    if (!has_voltage) {
        return std::nullopt;
    }
    return total_voltage;
}

std::optional<double> current_from_centiampere(const std::int16_t value) {
    if (value < 0) {
        return std::nullopt;
    }
    return static_cast<double>(value) / 100.0;
}

std::optional<std::int8_t> remaining_percent(const std::int8_t value) {
    if (value < 0) {
        return std::nullopt;
    }
    return value;
}

std::string status_text(const mavlink_statustext_t& message) {
    std::size_t length = 0;
    while (length < sizeof(message.text) && message.text[length] != '\0') {
        ++length;
    }
    return std::string(message.text, length);
}

std::string parameter_id(const mavlink_param_value_t& message) {
    std::size_t length = 0;
    while (
        length < sizeof(message.param_id) && message.param_id[length] != '\0') {
        ++length;
    }
    return std::string(message.param_id, length);
}

} // namespace

MavlinkDecoder::MavlinkDecoder(domain::VehicleState& state,
    CommandAckHandler command_ack_handler,
    MessageHandler message_handler,
    ParameterValueHandler parameter_value_handler)
    : state_(state), command_ack_handler_(std::move(command_ack_handler)),
      message_handler_(std::move(message_handler)),
      parameter_value_handler_(std::move(parameter_value_handler)) {}

void MavlinkDecoder::ingest(const std::span<const std::uint8_t> bytes,
    const domain::TimePoint now) {
    for (const auto byte : bytes) {
        mavlink_message_t parsed_message{};
        mavlink_status_t parsed_status{};
        const auto framing = mavlink_frame_char_buffer(&receive_message_,
            &receive_status_,
            byte,
            &parsed_message,
            &parsed_status);
        if (framing == MAVLINK_FRAMING_OK) {
            if (message_handler_) {
                const auto* info =
                    mavlink_get_message_info_by_id(parsed_message.msgid);
                message_handler_(
                    MessageObservation{
                        .message_id = parsed_message.msgid,
                        .source_system = parsed_message.sysid,
                        .source_component = parsed_message.compid,
                        .message_name = info == nullptr
                                            ? std::string_view{}
                                            : std::string_view{info->name},
                    },
                    now);
            }
            handle_message(parsed_message, now);
        }
    }
}

void MavlinkDecoder::handle_message(const mavlink_message_t& message,
    const domain::TimePoint now) {
    switch (message.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        handle_heartbeat(message, now);
        break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
        handle_gps(message, now);
        break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        handle_global_position(message, now);
        break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        handle_local_position(message, now);
        break;
    case MAVLINK_MSG_ID_ATTITUDE:
        handle_attitude(message, now);
        break;
    case MAVLINK_MSG_ID_BATTERY_STATUS:
        handle_battery(message, now);
        break;
    case MAVLINK_MSG_ID_SYS_STATUS:
        handle_system_status(message, now);
        break;
    case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
        handle_autopilot_version(message);
        break;
    case MAVLINK_MSG_ID_PARAM_VALUE:
        handle_parameter_value(message, now);
        break;
    case MAVLINK_MSG_ID_STATUSTEXT:
        handle_status_text(message, now);
        break;
    case MAVLINK_MSG_ID_COMMAND_ACK:
        handle_command_ack(message, now);
        break;
    default:
        break;
    }
}

void MavlinkDecoder::handle_heartbeat(const mavlink_message_t& message,
    const domain::TimePoint now) {
    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    if (heartbeat.autopilot == MAV_AUTOPILOT_INVALID) {
        return;
    }
    state_.on_heartbeat(message.sysid,
        message.compid,
        heartbeat.type,
        heartbeat.autopilot,
        heartbeat.base_mode,
        heartbeat.custom_mode,
        heartbeat.system_status,
        now);
}

void MavlinkDecoder::handle_gps(const mavlink_message_t& message,
    const domain::TimePoint now) {
    mavlink_gps_raw_int_t gps{};
    mavlink_msg_gps_raw_int_decode(&message, &gps);
    state_.on_gps(gps.fix_type, gps.satellites_visible, now);
}

void MavlinkDecoder::handle_global_position(const mavlink_message_t& message,
    const domain::TimePoint now) {
    mavlink_global_position_int_t position{};
    mavlink_msg_global_position_int_decode(&message, &position);
    state_.on_global_position(position.relative_alt, now);
}

void MavlinkDecoder::handle_local_position(const mavlink_message_t& message,
    const domain::TimePoint now) {
    mavlink_local_position_ned_t position{};
    mavlink_msg_local_position_ned_decode(&message, &position);
    state_.on_local_position(position.x, position.y, position.z, now);
}

void MavlinkDecoder::handle_attitude(const mavlink_message_t& message,
    const domain::TimePoint now) {
    mavlink_attitude_t attitude{};
    mavlink_msg_attitude_decode(&message, &attitude);
    state_.on_attitude(attitude.roll, attitude.pitch, attitude.yaw, now);
}

void MavlinkDecoder::handle_battery(const mavlink_message_t& message,
    const domain::TimePoint now) {
    mavlink_battery_status_t battery{};
    mavlink_msg_battery_status_decode(&message, &battery);
    state_.on_battery(battery_voltage(battery),
        current_from_centiampere(battery.current_battery),
        remaining_percent(battery.battery_remaining),
        now);
}

void MavlinkDecoder::handle_system_status(const mavlink_message_t& message,
    const domain::TimePoint now) {
    mavlink_sys_status_t status{};
    mavlink_msg_sys_status_decode(&message, &status);
    const auto voltage =
        status.voltage_battery > 0
            ? std::optional<double>{static_cast<double>(
                                        status.voltage_battery) /
                                    1000.0}
            : std::nullopt;
    state_.on_system_status(status.onboard_control_sensors_enabled,
        status.onboard_control_sensors_health,
        voltage,
        current_from_centiampere(status.current_battery),
        remaining_percent(status.battery_remaining),
        now);
}

void MavlinkDecoder::handle_autopilot_version(
    const mavlink_message_t& message) {
    if (message.compid != MAV_COMP_ID_AUTOPILOT1) {
        return;
    }
    mavlink_autopilot_version_t version{};
    mavlink_msg_autopilot_version_decode(&message, &version);
    const auto packed = version.flight_sw_version;
    state_.on_autopilot_metadata({
        .firmware_major = static_cast<std::uint8_t>((packed >> 24U) & 0xFFU),
        .firmware_minor = static_cast<std::uint8_t>((packed >> 16U) & 0xFFU),
        .firmware_patch = static_cast<std::uint8_t>((packed >> 8U) & 0xFFU),
        .firmware_release_type = static_cast<std::uint8_t>(packed & 0xFFU),
        .capabilities = version.capabilities,
        .board_version = version.board_version,
        .vendor_id = version.vendor_id,
        .product_id = version.product_id,
    });
}

void MavlinkDecoder::handle_parameter_value(const mavlink_message_t& message,
    const domain::TimePoint now) {
    mavlink_param_value_t parameter{};
    mavlink_msg_param_value_decode(&message, &parameter);
    const auto id = parameter_id(parameter);
    if (id == "BATT_ARM_VOLT") {
        state_.on_battery_arming_voltage(
            static_cast<double>(parameter.param_value),
            now);
    }
    if (!parameter_value_handler_) {
        return;
    }
    parameter_value_handler_(
        {
            .source_system = message.sysid,
            .source_component = message.compid,
            .id = id,
            .value = static_cast<double>(parameter.param_value),
            .type = parameter.param_type,
        },
        now);
}

void MavlinkDecoder::handle_status_text(const mavlink_message_t& message,
    const domain::TimePoint now) {
    mavlink_statustext_t status{};
    mavlink_msg_statustext_decode(&message, &status);
    state_.on_status_text(status.severity, status_text(status), now);
}

void MavlinkDecoder::handle_command_ack(const mavlink_message_t& message,
    const domain::TimePoint now) {
    if (!command_ack_handler_) {
        return;
    }
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message, &ack);
    command_ack_handler_(
        {
            .source_system = message.sysid,
            .source_component = message.compid,
            .command = ack.command,
            .result = ack.result,
            .progress = ack.progress,
            .result_parameter = ack.result_param2,
            .target_system = ack.target_system,
            .target_component = ack.target_component,
        },
        now);
}

} // namespace onboard_autonomy::adapters::mavlink
