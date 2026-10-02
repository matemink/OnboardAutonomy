#include "TestCases.hpp"

#include "onboard_autonomy/mission/CompanionApplication.hpp"
#include "onboard_autonomy/hardware/mavlink/MavlinkEncoder.hpp"

#include <ardupilotmega/mavlink.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <iterator>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::vector<std::uint8_t> serialize(const mavlink_message_t& message) {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto length = mavlink_msg_to_send_buffer(buffer.data(), &message);
    return {buffer.begin(), buffer.begin() + length};
}

class FakeTransport final : public onboard_autonomy::mission::ports::Transport {
  public:
    void enqueue(std::vector<std::uint8_t> frame) {
        incoming_.push_back(std::move(frame));
    }

    std::size_t read(const std::span<std::uint8_t> destination) override {
        if (incoming_.empty()) {
            return 0;
        }

        const auto frame = std::move(incoming_.front());
        incoming_.pop_front();
        require(frame.size() <= destination.size(),
            "fake input frame exceeds receive buffer");
        std::copy(frame.begin(), frame.end(), destination.begin());
        return frame.size();
    }

    std::size_t write(const std::span<const std::uint8_t> source) override {
        outgoing_.emplace_back(source.begin(), source.end());
        return source.size();
    }

    [[nodiscard]] std::string description() const override {
        return "fake://transport";
    }

    [[nodiscard]] const std::vector<std::vector<std::uint8_t>>&
    outgoing() const {
        return outgoing_;
    }

  private:
    std::deque<std::vector<std::uint8_t>> incoming_;
    std::vector<std::vector<std::uint8_t>> outgoing_;
};

class FakeCameraSource final
    : public onboard_autonomy::mission::ports::CameraSource {
  public:
    std::optional<onboard_autonomy::mission::ports::CameraFrame>
    take_latest_frame() override {
        return std::nullopt;
    }

    onboard_autonomy::mission::ports::CameraSourceStatus
    status() const override {
        onboard_autonomy::mission::ports::CameraSourceStatus result;
        result.phase =
            onboard_autonomy::mission::ports::CameraSourcePhase::streaming;
        result.description = "fake camera";
        return result;
    }
};

class FakeTargetDetector final
    : public onboard_autonomy::mission::ports::TargetDetector {
  public:
    onboard_autonomy::mission::TargetDetectionBatch detect(
        const onboard_autonomy::mission::ports::CameraFrame&) override {
        return {};
    }

    std::string description() const override { return "fake detector"; }
};

std::uint32_t message_id(const std::vector<std::uint8_t>& frame) {
    mavlink_message_t receive_buffer{};
    mavlink_status_t receive_status{};
    mavlink_message_t parsed_message{};
    mavlink_status_t parsed_status{};

    for (const auto byte : frame) {
        if (mavlink_frame_char_buffer(&receive_buffer,
                &receive_status,
                byte,
                &parsed_message,
                &parsed_status) == MAVLINK_FRAMING_OK) {
            return parsed_message.msgid;
        }
    }
    throw std::runtime_error("application emitted an invalid frame");
}

std::size_t count_messages(const std::vector<std::vector<std::uint8_t>>& frames,
    const std::uint32_t expected_id) {
    return static_cast<std::size_t>(std::count_if(frames.begin(),
        frames.end(),
        [expected_id](
            const auto& frame) { return message_id(frame) == expected_id; }));
}

std::string parameter_request_id(const std::vector<std::uint8_t>& frame) {
    mavlink_message_t receive_buffer{};
    mavlink_status_t receive_status{};
    mavlink_message_t parsed_message{};
    mavlink_status_t parsed_status{};
    for (const auto byte : frame) {
        if (mavlink_frame_char_buffer(&receive_buffer,
                &receive_status,
                byte,
                &parsed_message,
                &parsed_status) == MAVLINK_FRAMING_OK) {
            break;
        }
    }
    require(parsed_message.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ,
        "frame must contain PARAM_REQUEST_READ");
    mavlink_param_request_read_t request{};
    mavlink_msg_param_request_read_decode(&parsed_message, &request);
    const auto end = std::find(std::begin(request.param_id),
        std::end(request.param_id),
        '\0');
    return {std::begin(request.param_id), end};
}

std::vector<std::uint8_t> autopilot_heartbeat(const bool armed = false) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(1,
        MAV_COMP_ID_AUTOPILOT1,
        &message,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0,
        0,
        MAV_STATE_STANDBY);
    return serialize(message);
}

std::optional<std::uint16_t> command_long_command(
    const std::vector<std::uint8_t>& frame) {
    mavlink_message_t receive_buffer{};
    mavlink_status_t receive_status{};
    mavlink_message_t parsed_message{};
    mavlink_status_t parsed_status{};
    for (const auto byte : frame) {
        if (mavlink_frame_char_buffer(&receive_buffer,
                &receive_status,
                byte,
                &parsed_message,
                &parsed_status) != MAVLINK_FRAMING_OK) {
            continue;
        }
        if (parsed_message.msgid != MAVLINK_MSG_ID_COMMAND_LONG) {
            return std::nullopt;
        }
        mavlink_command_long_t command{};
        mavlink_msg_command_long_decode(&parsed_message, &command);
        return command.command;
    }
    return std::nullopt;
}

std::vector<std::uint8_t> accepted_interval_ack() {
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack(1,
        MAV_COMP_ID_AUTOPILOT1,
        &message,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        MAV_RESULT_ACCEPTED,
        100,
        0,
        1,
        onboard_autonomy::hardware::mavlink::kCompanionComponentId);
    return serialize(message);
}

void application_orchestrates_the_complete_telemetry_setup() {
    FakeTransport transport;
    onboard_autonomy::mission::CompanionApplication application{transport};
    const onboard_autonomy::mission::TimePoint start{};

    transport.enqueue(autopilot_heartbeat());
    application.poll(start);

    auto snapshot = application.snapshot(start);
    require(snapshot.vehicle.connected, "application must decode heartbeat");
    require(snapshot.companion_heartbeat_active,
        "application must publish its companion heartbeat");
    require(snapshot.telemetry.state ==
                onboard_autonomy::mission::TelemetrySetupState::configuring,
        "application must start telemetry setup");
    require(
        transport.outgoing().size() == 2 &&
            message_id(transport.outgoing()[0]) == MAVLINK_MSG_ID_HEARTBEAT &&
            message_id(transport.outgoing()[1]) == MAVLINK_MSG_ID_COMMAND_LONG,
        "application must emit heartbeat and first setup command");
    require(snapshot.link_events.size() == 2 &&
                snapshot.link_events[0].direction ==
                    onboard_autonomy::mission::LinkEventDirection::inbound &&
                snapshot.link_events[0].label == "HEARTBEAT" &&
                snapshot.link_events[1].direction ==
                    onboard_autonomy::mission::LinkEventDirection::outbound &&
                snapshot.link_events[1].label == "SET_INTERVAL",
        "application must expose real inbound and outbound link events");
    require(snapshot.rx_activity.has_value() &&
                snapshot.rx_activity->message_name == "HEARTBEAT" &&
                snapshot.tx_activity.has_value() &&
                snapshot.tx_activity->message_name == "COMMAND_LONG" &&
                snapshot.tx_activity->detail.find("SYS_STATUS") !=
                    std::string::npos,
        "live activity must report the actual received heartbeat and "
        "latest transmitted MAVLink frame");

    for (int index = 0; index < 6; ++index) {
        transport.enqueue(accepted_interval_ack());
        application.poll(start + std::chrono::milliseconds((index + 1) * 100));
    }

    snapshot = application.snapshot(start + std::chrono::milliseconds(600));
    require(snapshot.telemetry.state ==
                onboard_autonomy::mission::TelemetrySetupState::active,
        "six accepted commands must activate telemetry");
    require(transport.outgoing().size() == 10 &&
                message_id(transport.outgoing()[7]) ==
                    MAVLINK_MSG_ID_PARAM_REQUEST_READ &&
                parameter_request_id(transport.outgoing()[7]) ==
                    "FS_GCS_ENABLE" &&
                message_id(transport.outgoing()[8]) ==
                    MAVLINK_MSG_ID_COMMAND_LONG &&
                message_id(transport.outgoing().back()) ==
                    MAVLINK_MSG_ID_PARAM_REQUEST_READ,
        "application must request failsafe policy, version, and battery "
        "threshold after setup");
    require(snapshot.link_events.size() <= 8,
        "link event history must remain bounded");
    require(std::any_of(snapshot.link_events.begin(),
                snapshot.link_events.end(),
                [](const onboard_autonomy::mission::LinkEvent& event) {
                    return event.direction == onboard_autonomy::mission::
                                                  LinkEventDirection::inbound &&
                           event.status == onboard_autonomy::mission::
                                               LinkEventStatus::success &&
                           event.label == "ACK SET_INTERVAL";
                }),
        "accepted COMMAND_ACK must appear in the link event history");
    require(snapshot.rx_activity.has_value() &&
                snapshot.rx_activity->message_name == "COMMAND_ACK" &&
                snapshot.rx_activity->detail.find("ACCEPTED") !=
                    std::string::npos &&
                snapshot.tx_activity.has_value() &&
                snapshot.tx_activity->message_name == "PARAM_REQUEST_READ" &&
                snapshot.tx_activity->detail == "BATT_ARM_VOLT",
        "live activity must follow the newest real RX and TX frames");

    for (const auto& frame : transport.outgoing()) {
        const auto id = message_id(frame);
        require(id == MAVLINK_MSG_ID_HEARTBEAT ||
                    id == MAVLINK_MSG_ID_PARAM_REQUEST_READ ||
                    id == MAVLINK_MSG_ID_COMMAND_LONG,
            "observation runtime must not emit flight-control messages");
        if (const auto command = command_long_command(frame)) {
            require(*command == MAV_CMD_SET_MESSAGE_INTERVAL ||
                        *command == MAV_CMD_REQUEST_MESSAGE,
                "COMMAND_LONG must only configure or request telemetry");
        }
    }
}

void quiet_transport_does_not_stall_runtime_scheduling() {
    FakeTransport transport;
    onboard_autonomy::mission::CompanionApplication application{transport};
    const onboard_autonomy::mission::TimePoint start{};

    transport.enqueue(autopilot_heartbeat());
    application.poll(start);
    const auto initial_heartbeats =
        count_messages(transport.outgoing(), MAVLINK_MSG_ID_HEARTBEAT);

    application.poll(start + std::chrono::seconds(1));
    const auto later_heartbeats =
        count_messages(transport.outgoing(), MAVLINK_MSG_ID_HEARTBEAT);

    require(initial_heartbeats == 1 && later_heartbeats == 2,
        "quiet transport must not stop scheduled companion heartbeat");
}

} // namespace

void run_companion_application_tests() {
    application_orchestrates_the_complete_telemetry_setup();
    quiet_transport_does_not_stall_runtime_scheduling();
}
