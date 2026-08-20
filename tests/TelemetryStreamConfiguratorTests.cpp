#include "TestCases.hpp"

#include "onboard_autonomy/hardware/mavlink/MavlinkEncoder.hpp"
#include "onboard_autonomy/hardware/mavlink/TelemetryStreamConfigurator.hpp"

#include <ardupilotmega/mavlink.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

mavlink_command_long_t decode_command(const std::vector<std::uint8_t>& bytes) {
    mavlink_message_t receive_buffer{};
    mavlink_status_t receive_status{};
    mavlink_message_t parsed_message{};
    mavlink_status_t parsed_status{};

    for (const auto byte : bytes) {
        const auto framing = mavlink_frame_char_buffer(&receive_buffer,
            &receive_status,
            byte,
            &parsed_message,
            &parsed_status);
        if (framing == MAVLINK_FRAMING_OK) {
            require(parsed_message.msgid == MAVLINK_MSG_ID_COMMAND_LONG,
                "configurator must emit COMMAND_LONG");
            mavlink_command_long_t command{};
            mavlink_msg_command_long_decode(&parsed_message, &command);
            return command;
        }
    }

    throw std::runtime_error("configurator emitted an invalid frame");
}

onboard_autonomy::hardware::mavlink::CommandAck accepted_ack() {
    return {
        .source_system = 1,
        .source_component = MAV_COMP_ID_AUTOPILOT1,
        .command = MAV_CMD_SET_MESSAGE_INTERVAL,
        .result = MAV_RESULT_ACCEPTED,
        .progress = 100,
        .result_parameter = 0,
        .target_system = 1,
        .target_component =
            onboard_autonomy::hardware::mavlink::kCompanionComponentId,
    };
}

void requests_each_required_stream_sequentially() {
    onboard_autonomy::hardware::mavlink::TelemetryStreamConfigurator
        configurator;
    const onboard_autonomy::mission::TimePoint now{};

    require(!configurator.update(false, std::nullopt, now).has_value(),
        "disconnected configurator must not emit commands");

    const auto system_health = configurator.update(true, 1, now);
    require(system_health.has_value(), "first request must be emitted");
    require(decode_command(*system_health).param1 ==
                static_cast<float>(MAVLINK_MSG_ID_SYS_STATUS),
        "SYS_STATUS must be requested first");
    require(!configurator.update(true, 1, now).has_value(),
        "next request must wait for COMMAND_ACK");

    configurator.on_command_ack(accepted_ack(), now);
    const auto gps = configurator.update(true, 1, now);
    require(gps.has_value(), "GPS request must follow first ACK");
    require(decode_command(*gps).param1 ==
                static_cast<float>(MAVLINK_MSG_ID_GPS_RAW_INT),
        "GPS_RAW_INT must be requested second");

    configurator.on_command_ack(accepted_ack(), now);
    const auto battery = configurator.update(true, 1, now);
    require(battery.has_value(), "battery request must follow second ACK");
    require(decode_command(*battery).param1 ==
                static_cast<float>(MAVLINK_MSG_ID_BATTERY_STATUS),
        "BATTERY_STATUS must be requested third");

    configurator.on_command_ack(accepted_ack(), now);
    const auto position = configurator.update(true, 1, now);
    require(position.has_value(), "position request must follow battery ACK");
    require(decode_command(*position).param1 ==
                static_cast<float>(MAVLINK_MSG_ID_GLOBAL_POSITION_INT),
        "GLOBAL_POSITION_INT must be requested fourth");

    configurator.on_command_ack(accepted_ack(), now);
    const auto local_position = configurator.update(true, 1, now);
    require(local_position.has_value(),
        "local position request must follow global position ACK");
    require(decode_command(*local_position).param1 ==
                static_cast<float>(MAVLINK_MSG_ID_LOCAL_POSITION_NED),
        "LOCAL_POSITION_NED must be requested fifth");

    configurator.on_command_ack(accepted_ack(), now);
    const auto attitude = configurator.update(true, 1, now);
    require(attitude.has_value(),
        "attitude request must follow local position ACK");
    require(decode_command(*attitude).param1 ==
                static_cast<float>(MAVLINK_MSG_ID_ATTITUDE),
        "ATTITUDE must be requested sixth");

    configurator.on_command_ack(accepted_ack(), now);
    const auto snapshot = configurator.snapshot();
    require(
        snapshot.phase ==
            onboard_autonomy::hardware::mavlink::TelemetrySetupPhase::active,
        "all accepted requests must activate telemetry");
    require(snapshot.completed_requests == snapshot.total_requests,
        "all requests must be counted");
}

void retries_then_fails_when_ack_is_missing() {
    onboard_autonomy::hardware::mavlink::TelemetryStreamConfigurator
        configurator;
    const onboard_autonomy::mission::TimePoint start{};

    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto now = start + std::chrono::seconds(attempt * 3);
        const auto frame = configurator.update(true, 1, now);
        require(frame.has_value(), "each timeout must trigger a retry");
        const auto command = decode_command(*frame);
        require(command.confirmation == attempt,
            "retry confirmation must increase");
    }

    const auto after_retries = start + std::chrono::seconds(9);
    require(!configurator.update(true, 1, after_retries).has_value(),
        "configurator must stop after the retry budget");
    const auto snapshot = configurator.snapshot();
    require(
        snapshot.phase ==
            onboard_autonomy::hardware::mavlink::TelemetrySetupPhase::failed,
        "missing ACK must fail setup");
    require(!snapshot.failure_result.has_value(),
        "timeout must be distinct from a rejected command");
}

void rejected_command_fails_immediately() {
    onboard_autonomy::hardware::mavlink::TelemetryStreamConfigurator
        configurator;
    const onboard_autonomy::mission::TimePoint now{};
    require(configurator.update(true, 1, now).has_value(),
        "request must be in flight before its ACK");

    auto acknowledgement = accepted_ack();
    acknowledgement.result = MAV_RESULT_UNSUPPORTED;
    configurator.on_command_ack(acknowledgement, now);

    const auto snapshot = configurator.snapshot();
    require(
        snapshot.phase ==
            onboard_autonomy::hardware::mavlink::TelemetrySetupPhase::failed,
        "rejected request must fail setup");
    require(snapshot.failure_result == MAV_RESULT_UNSUPPORTED,
        "MAV_RESULT must be preserved for diagnostics");
}

void reconnect_restarts_the_complete_stream_sequence() {
    onboard_autonomy::hardware::mavlink::TelemetryStreamConfigurator
        configurator;
    const onboard_autonomy::mission::TimePoint now{};

    for (int request = 0; request < 6; ++request) {
        require(configurator.update(true, 1, now).has_value(),
            "initial stream sequence must emit every request");
        configurator.on_command_ack(accepted_ack(), now);
    }
    require(
        configurator.snapshot().phase ==
            onboard_autonomy::hardware::mavlink::TelemetrySetupPhase::active,
        "initial stream sequence must become active");

    require(!configurator.update(false, 1, now).has_value(),
        "disconnect must not emit telemetry commands");
    const auto disconnected = configurator.snapshot();
    require(disconnected.phase ==
                    onboard_autonomy::hardware::mavlink::TelemetrySetupPhase::
                        waiting_for_vehicle &&
                disconnected.completed_requests == 0,
        "disconnect must clear the previous telemetry session");

    const auto first_reconnected_request = configurator.update(true, 1, now);
    require(first_reconnected_request.has_value() &&
                decode_command(*first_reconnected_request).param1 ==
                    static_cast<float>(MAVLINK_MSG_ID_SYS_STATUS),
        "reconnect must restart configuration from SYS_STATUS");
}

} // namespace

void run_telemetry_stream_configurator_tests() {
    requests_each_required_stream_sequentially();
    retries_then_fails_when_ack_is_missing();
    rejected_command_fails_immediately();
    reconnect_restarts_the_complete_stream_sequence();
}
