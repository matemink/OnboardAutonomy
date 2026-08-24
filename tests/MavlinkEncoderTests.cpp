#include "TestCases.hpp"

#include "onboard_autonomy/hardware/mavlink/MavlinkEncoder.hpp"

#include <ardupilotmega/mavlink.h>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

mavlink_command_long_t decode_command_long(
    const std::vector<std::uint8_t>& bytes) {
    mavlink_message_t receive_buffer{};
    mavlink_status_t receive_status{};
    mavlink_message_t parsed_message{};
    mavlink_status_t parsed_status{};

    for (const auto byte : bytes) {
        if (mavlink_frame_char_buffer(&receive_buffer,
                &receive_status,
                byte,
                &parsed_message,
                &parsed_status) == MAVLINK_FRAMING_OK) {
            require(parsed_message.msgid == MAVLINK_MSG_ID_COMMAND_LONG,
                "flight command must use COMMAND_LONG");
            mavlink_command_long_t command{};
            mavlink_msg_command_long_decode(&parsed_message, &command);
            return command;
        }
    }

    throw std::runtime_error("flight command must be a valid MAVLink frame");
}

mavlink_message_t decode_message(const std::vector<std::uint8_t>& bytes) {
    mavlink_message_t receive_buffer{};
    mavlink_status_t receive_status{};
    mavlink_message_t parsed_message{};
    mavlink_status_t parsed_status{};

    for (const auto byte : bytes) {
        if (mavlink_frame_char_buffer(&receive_buffer,
                &receive_status,
                byte,
                &parsed_message,
                &parsed_status) == MAVLINK_FRAMING_OK) {
            return parsed_message;
        }
    }
    throw std::runtime_error("expected a valid MAVLink frame");
}

void companion_heartbeat_uses_standard_identity() {
    constexpr std::uint8_t system_id{42};
    const auto bytes =
        onboard_autonomy::hardware::mavlink::encode_companion_heartbeat(
            system_id);

    mavlink_message_t receive_buffer{};
    mavlink_status_t receive_status{};
    mavlink_message_t parsed_message{};
    mavlink_status_t parsed_status{};
    bool frame_complete = false;

    for (const auto byte : bytes) {
        const auto framing = mavlink_frame_char_buffer(&receive_buffer,
            &receive_status,
            byte,
            &parsed_message,
            &parsed_status);
        frame_complete = frame_complete || framing == MAVLINK_FRAMING_OK;
    }

    require(frame_complete, "encoded heartbeat must be a valid frame");
    require(parsed_message.msgid == MAVLINK_MSG_ID_HEARTBEAT, "wrong msgid");
    require(parsed_message.sysid == system_id, "wrong system id");
    require(parsed_message.compid ==
                onboard_autonomy::hardware::mavlink::kCompanionComponentId,
        "wrong component id");

    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&parsed_message, &heartbeat);
    require(heartbeat.type == MAV_TYPE_ONBOARD_CONTROLLER,
        "companion must identify as an onboard controller");
    require(heartbeat.autopilot == MAV_AUTOPILOT_INVALID,
        "companion must not identify as a flight controller");
    require(heartbeat.system_status == MAV_STATE_ACTIVE,
        "running companion must report active state");
}

void message_interval_request_uses_command_long() {
    constexpr std::uint8_t system_id{7};
    constexpr std::uint32_t interval_microseconds{500'000};
    const auto bytes =
        onboard_autonomy::hardware::mavlink::encode_set_message_interval(
            system_id,
            MAVLINK_MSG_ID_GPS_RAW_INT,
            interval_microseconds);

    mavlink_message_t receive_buffer{};
    mavlink_status_t receive_status{};
    mavlink_message_t parsed_message{};
    mavlink_status_t parsed_status{};
    bool frame_complete = false;

    for (const auto byte : bytes) {
        const auto framing = mavlink_frame_char_buffer(&receive_buffer,
            &receive_status,
            byte,
            &parsed_message,
            &parsed_status);
        frame_complete = frame_complete || framing == MAVLINK_FRAMING_OK;
    }

    require(frame_complete, "message interval request must be valid");
    require(parsed_message.msgid == MAVLINK_MSG_ID_COMMAND_LONG,
        "message interval request must use COMMAND_LONG");
    require(parsed_message.sysid == system_id, "wrong source system");
    require(parsed_message.compid ==
                onboard_autonomy::hardware::mavlink::kCompanionComponentId,
        "wrong source component");

    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&parsed_message, &command);
    require(command.target_system == system_id,
        "command must target the discovered vehicle");
    require(command.target_component == 0,
        "message interval command must use the autopilot default target");
    require(command.command == MAV_CMD_SET_MESSAGE_INTERVAL, "wrong MAV_CMD");
    require(command.param1 == static_cast<float>(MAVLINK_MSG_ID_GPS_RAW_INT),
        "param1 must contain the requested message ID");
    require(command.param2 == static_cast<float>(interval_microseconds),
        "param2 must contain the interval in microseconds");
}

void battery_threshold_request_uses_parameter_protocol() {
    constexpr std::uint8_t system_id{7};
    const auto message = decode_message(onboard_autonomy::hardware::mavlink::
            encode_battery_arming_voltage_request(system_id));

    require(message.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ,
        "battery threshold must use PARAM_REQUEST_READ");
    require(message.sysid == system_id, "wrong source system");
    require(message.compid ==
                onboard_autonomy::hardware::mavlink::kCompanionComponentId,
        "wrong source component");

    mavlink_param_request_read_t request{};
    mavlink_msg_param_request_read_decode(&message, &request);
    require(request.target_system == system_id &&
                request.target_component == MAV_COMP_ID_AUTOPILOT1,
        "battery parameter request must target the autopilot");
    require(std::string(request.param_id,
                request.param_id + sizeof(request.param_id))
                .starts_with("BATT_ARM_VOLT"),
        "battery parameter request must name BATT_ARM_VOLT");
    require(request.param_index == -1,
        "named parameter request must not use an index");
}

void named_parameter_request_uses_parameter_protocol() {
    constexpr std::uint8_t system_id{1};
    const auto message = decode_message(
        onboard_autonomy::hardware::mavlink::encode_parameter_request_read(
            system_id,
            "FS_GCS_TIMEOUT"));
    mavlink_param_request_read_t request{};
    mavlink_msg_param_request_read_decode(&message, &request);

    require(message.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ &&
                request.target_system == system_id &&
                request.target_component == MAV_COMP_ID_AUTOPILOT1 &&
                request.param_index == -1,
        "named parameter read must target the active autopilot");
    require(std::string(request.param_id,
                request.param_id + sizeof(request.param_id))
                .starts_with("FS_GCS_TIMEOUT"),
        "named parameter read must preserve its 16-byte id");
}

void autopilot_version_uses_one_shot_message_request() {
    constexpr std::uint8_t system_id{7};
    const auto command = decode_command_long(
        onboard_autonomy::hardware::mavlink::encode_autopilot_version_request(
            system_id));

    require(command.command == MAV_CMD_REQUEST_MESSAGE,
        "autopilot version must use MAV_CMD_REQUEST_MESSAGE");
    require(command.target_system == system_id && command.target_component == 0,
        "version request must target the discovered flight stack");
    require(command.param1 ==
                static_cast<float>(MAVLINK_MSG_ID_AUTOPILOT_VERSION),
        "param1 must contain AUTOPILOT_VERSION message ID");
    require(command.param7 == 1.0F,
        "version response must be addressed back to the requester");
}

void flight_commands_use_documented_arducopter_parameters() {
    constexpr std::uint8_t system_id{1};

    const auto guided = decode_command_long(
        onboard_autonomy::hardware::mavlink::encode_set_guided_mode(system_id));
    require(guided.command == MAV_CMD_DO_SET_MODE &&
                guided.param1 ==
                    static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) &&
                guided.param2 == 4.0F,
        "GUIDED must use MAV_CMD_DO_SET_MODE and Copter mode 4");

    const auto arm = decode_command_long(
        onboard_autonomy::hardware::mavlink::encode_arm(system_id));
    require(arm.command == MAV_CMD_COMPONENT_ARM_DISARM && arm.param1 == 1.0F &&
                arm.param2 == 0.0F,
        "arm must preserve ArduPilot safety checks");

    const auto takeoff = decode_command_long(
        onboard_autonomy::hardware::mavlink::encode_takeoff(system_id, 5.0));
    require(takeoff.command == MAV_CMD_NAV_TAKEOFF && takeoff.param7 == 5.0F,
        "takeoff altitude must be encoded in param7");

    const auto land = decode_command_long(
        onboard_autonomy::hardware::mavlink::encode_land(system_id));
    require(land.command == MAV_CMD_NAV_LAND,
        "landing must use MAV_CMD_NAV_LAND");

    const auto rtl = decode_command_long(
        onboard_autonomy::hardware::mavlink::encode_return_to_launch(
            system_id));
    require(rtl.command == MAV_CMD_NAV_RETURN_TO_LAUNCH,
        "RTL must use MAV_CMD_NAV_RETURN_TO_LAUNCH");

    const auto yaw = decode_command_long(
        onboard_autonomy::hardware::mavlink::encode_condition_yaw(system_id,
            -8.0,
            15.0));
    require(yaw.command == MAV_CMD_CONDITION_YAW && yaw.param1 == 8.0F &&
                yaw.param2 == 15.0F && yaw.param3 == -1.0F &&
                yaw.param4 == 1.0F,
        "yaw guidance must encode a bounded relative counter-clockwise turn");
}

void movement_and_precision_messages_use_documented_frames() {
    constexpr std::uint8_t system_id{1};
    const auto move_message = decode_message(
        onboard_autonomy::hardware::mavlink::encode_local_position_target(
            system_id,
            10.0,
            -4.0,
            0.0));
    require(move_message.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED,
        "route step must use SET_POSITION_TARGET_LOCAL_NED");
    mavlink_set_position_target_local_ned_t move{};
    mavlink_msg_set_position_target_local_ned_decode(&move_message, &move);
    require(move.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED &&
                move.type_mask == 3576 && move.x == 10.0F && move.y == -4.0F &&
                move.z == 0.0F,
        "route target must be a documented local NED position offset");

    const auto target_message = decode_message(
        onboard_autonomy::hardware::mavlink::encode_landing_target(system_id,
            123'000,
            -8.0,
            -4.0,
            8.0));
    require(target_message.msgid == MAVLINK_MSG_ID_LANDING_TARGET,
        "precision step must use LANDING_TARGET");
    mavlink_landing_target_t target{};
    mavlink_msg_landing_target_decode(&target_message, &target);
    require(target.frame == MAV_FRAME_BODY_FRD && target.position_valid == 1 &&
                target.type == LANDING_TARGET_TYPE_VISION_FIDUCIAL &&
                target.x == -8.0F && target.y == -4.0F && target.z == 8.0F,
        "precision target must contain a valid body-FRD position");
}

} // namespace

void run_mavlink_encoder_tests() {
    companion_heartbeat_uses_standard_identity();
    message_interval_request_uses_command_long();
    battery_threshold_request_uses_parameter_protocol();
    named_parameter_request_uses_parameter_protocol();
    autopilot_version_uses_one_shot_message_request();
    flight_commands_use_documented_arducopter_parameters();
    movement_and_precision_messages_use_documented_frames();
}
