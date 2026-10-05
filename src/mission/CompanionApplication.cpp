#include "onboard_autonomy/mission/CompanionApplication.hpp"

#include "onboard_autonomy/hardware/mavlink/MavlinkDecoder.hpp"
#include "onboard_autonomy/hardware/mavlink/MavlinkEncoder.hpp"
#include "onboard_autonomy/hardware/mavlink/TelemetryStreamConfigurator.hpp"
#include "onboard_autonomy/mission/safety/CompanionLinkFailsafe.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace onboard_autonomy::mission {
namespace {

constexpr std::uint32_t kStabilizeMode = 0;
constexpr std::uint32_t kAutoMode = 3;
constexpr std::uint32_t kGuidedMode = 4;
constexpr std::uint32_t kLoiterMode = 5;
constexpr std::uint32_t kReturnToLaunchMode = 6;
constexpr std::uint32_t kLandMode = 9;
constexpr std::uint32_t kPositionHoldMode = 16;
constexpr std::size_t kTransportReceiveBufferSize = 4096;

TelemetrySetupState map_telemetry_state(
    const hardware::mavlink::TelemetrySetupPhase phase) {
    switch (phase) {
    case hardware::mavlink::TelemetrySetupPhase::waiting_for_vehicle:
        return TelemetrySetupState::waiting_for_vehicle;
    case hardware::mavlink::TelemetrySetupPhase::configuring:
        return TelemetrySetupState::configuring;
    case hardware::mavlink::TelemetrySetupPhase::active:
        return TelemetrySetupState::active;
    case hardware::mavlink::TelemetrySetupPhase::failed:
        return TelemetrySetupState::failed;
    }
    return TelemetrySetupState::failed;
}

LinkEventStatus map_event_status(const std::uint8_t result) {
    switch (result) {
    case MAV_RESULT_ACCEPTED:
        return LinkEventStatus::success;
    case MAV_RESULT_IN_PROGRESS:
        return LinkEventStatus::pending;
    case MAV_RESULT_TEMPORARILY_REJECTED:
        return LinkEventStatus::warning;
    default:
        return LinkEventStatus::failure;
    }
}

std::string ack_result_name(const std::uint8_t result) {
    switch (result) {
    case MAV_RESULT_ACCEPTED:
        return "ACCEPTED";
    case MAV_RESULT_TEMPORARILY_REJECTED:
        return "TEMPORARILY REJECTED";
    case MAV_RESULT_DENIED:
        return "DENIED";
    case MAV_RESULT_UNSUPPORTED:
        return "UNSUPPORTED";
    case MAV_RESULT_FAILED:
        return "FAILED";
    case MAV_RESULT_IN_PROGRESS:
        return "IN PROGRESS";
    case MAV_RESULT_CANCELLED:
        return "CANCELLED";
    default:
        return "MAV_RESULT " + std::to_string(result);
    }
}

std::string mav_command_name(const std::uint16_t command) {
    switch (command) {
    case MAV_CMD_SET_MESSAGE_INTERVAL:
        return "ACK SET_INTERVAL";
    case MAV_CMD_REQUEST_MESSAGE:
        return "ACK REQUEST_VERSION";
    case MAV_CMD_DO_SET_MODE:
        return "ACK SET_MODE";
    case MAV_CMD_COMPONENT_ARM_DISARM:
        return "ACK ARM";
    case MAV_CMD_NAV_TAKEOFF:
        return "ACK TAKEOFF";
    case MAV_CMD_NAV_LAND:
        return "ACK LAND";
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return "ACK RTL";
    case MAV_CMD_CONDITION_YAW:
        return "ACK YAW";
    default:
        return "ACK COMMAND " + std::to_string(command);
    }
}

std::string command_ack_activity_detail(const std::uint16_t command,
    const std::uint8_t result) {
    std::string command_name = mav_command_name(command);
    if (command_name.starts_with("ACK ")) {
        command_name.erase(0, 4);
    }
    return command_name + " " + ack_result_name(result);
}

std::string flight_mode_name(const std::uint32_t mode) {
    switch (mode) {
    case kStabilizeMode:
        return "STABILIZE";
    case kAutoMode:
        return "AUTO";
    case kGuidedMode:
        return "GUIDED";
    case kLoiterMode:
        return "LOITER";
    case kReturnToLaunchMode:
        return "RTL";
    case kLandMode:
        return "LAND";
    case kPositionHoldMode:
        return "POSITION HOLD";
    default:
        return "MODE " + std::to_string(mode);
    }
}

} // namespace

class CompanionApplication::Impl {
  public:
    explicit Impl(ports::Transport& transport,
        CompanionApplicationOptions options)
        : transport_(transport),
          simulated_wind_(options.simulated_wind),
          decoder_{
              vehicle_state_,
              [this](const hardware::mavlink::CommandAck& acknowledgement,
                  const mission::TimePoint now) {
                  const auto telemetry_before =
                      telemetry_configurator_.snapshot();

                  if (acknowledgement.target_component == 0 ||
                      acknowledgement.target_component ==
                          hardware::mavlink::kCompanionComponentId) {
                      std::string detail =
                          ack_result_name(acknowledgement.result);
                      if (acknowledgement.command ==
                              MAV_CMD_SET_MESSAGE_INTERVAL &&
                          telemetry_before.phase ==
                              hardware::mavlink::TelemetrySetupPhase::
                                  configuring &&
                          !telemetry_before.current_stream.empty()) {
                          detail =
                              telemetry_before.current_stream + " | " + detail;
                      }
                      detail += " | SYS " +
                                std::to_string(acknowledgement.source_system);
                      record_event(LinkEventDirection::inbound,
                          map_event_status(acknowledgement.result),
                          mav_command_name(acknowledgement.command),
                          std::move(detail),
                          now);
                      record_activity(LinkEventDirection::inbound,
                          "COMMAND_ACK",
                          command_ack_activity_detail(acknowledgement.command,
                              acknowledgement.result),
                          now);
                  }

                  telemetry_configurator_.on_command_ack(acknowledgement, now);
              },
              [this](const hardware::mavlink::MessageObservation& message,
                  const mission::TimePoint now) {
                  record_activity(LinkEventDirection::inbound,
                      message.message_name.empty()
                          ? "MSG #" + std::to_string(message.message_id)
                          : std::string(message.message_name),
                      "",
                      now);
              },
              [this](const hardware::mavlink::ParameterValue& parameter,
                  const mission::TimePoint now) {
                  companion_link_failsafe_.on_parameter(parameter.source_system,
                      parameter.source_component,
                      parameter.id,
                      parameter.value);
                  observe_failsafe_phase(now);
                  for (const auto expected :
                      CompanionLinkFailsafe::parameter_names) {
                      if (parameter.id == expected) {
                          record_activity(LinkEventDirection::inbound,
                              "PARAM_VALUE",
                              parameter.id,
                              now);
                          break;
                      }
                  }
              },
          } {
        if (options.camera_source != nullptr) {
            camera_monitor_.emplace(*options.camera_source,
                options.target_detector);
        } else if (options.target_detector != nullptr) {
            throw std::invalid_argument("vision requires a camera source");
        }
    }

    void poll() { poll(std::nullopt); }

    void poll(const mission::TimePoint now) { poll(std::optional{now}); }

    void set_observation_sinks(std::vector<ports::RuntimeSnapshotSink*> sinks) {
        observation_sinks_ = std::move(sinks);
    }

  private:
    mission::TimePoint poll_inputs(
        const std::optional<mission::TimePoint>& fixed_now) {
        const auto camera_now = fixed_now.value_or(mission::Clock::now());
        if (!started_at_.has_value()) {
            started_at_ = camera_now;
        }
        if (camera_monitor_.has_value()) {
            camera_monitor_->poll(camera_now);
        }
        const std::size_t received = transport_.read(receive_buffer_);
        const auto now = fixed_now.value_or(mission::Clock::now());
        if (received > 0U) {
            decoder_.ingest(
                std::span<const std::uint8_t>{receive_buffer_.data(), received},
                now);
        }
        return now;
    }

    void update_vehicle_connection(const mission::VehicleSnapshot& vehicle,
        const mission::TimePoint now) {
        const bool newly_connected =
            vehicle.connected && !vehicle_was_connected_;
        companion_link_failsafe_.observe_vehicle(vehicle.connected,
            vehicle.system_id);
        observe_failsafe_phase(now);
        if (newly_connected) {
            failsafe_parameter_index_ = 0;
            next_failsafe_parameter_request_ = now;
        }
        observe_vehicle(vehicle, now);
        if (!vehicle.connected) {
            companion_heartbeat_active_ = false;
        }
    }

    void send_companion_heartbeat(const mission::VehicleSnapshot& vehicle,
        const mission::TimePoint now) {
        if (now < next_heartbeat_) {
            return;
        }
        if (vehicle.connected && vehicle.system_id.has_value()) {
            const auto frame = hardware::mavlink::encode_companion_heartbeat(
                vehicle.system_id.value());
            if (write_frame(frame, now, "HEARTBEAT")) {
                companion_heartbeat_active_ = true;
            }
        }
        next_heartbeat_ = now + kHeartbeatInterval;
    }

    bool update_telemetry_setup(const mission::VehicleSnapshot& vehicle,
        const mission::TimePoint now) {
        const auto frame = telemetry_configurator_.update(vehicle.connected,
            vehicle.system_id,
            now);
        if (frame.has_value()) {
            const auto setup = telemetry_configurator_.snapshot();
            const bool sent = write_frame(frame.value(),
                now,
                "COMMAND_LONG",
                "SET_INTERVAL " + setup.current_stream);
            record_event(LinkEventDirection::outbound,
                sent ? LinkEventStatus::pending : LinkEventStatus::failure,
                "SET_INTERVAL",
                setup.current_stream + " | ATTEMPT " +
                    std::to_string(setup.attempt) +
                    (sent ? "" : " | WRITE FAILED"),
                now);
        }
        return telemetry_configurator_.snapshot().phase ==
               hardware::mavlink::TelemetrySetupPhase::active;
    }

    void request_failsafe_parameter(const mission::VehicleSnapshot& vehicle,
        const bool telemetry_ready,
        const mission::TimePoint now) {
        if (!telemetry_ready || !vehicle.system_id.has_value() ||
            now < next_failsafe_parameter_request_) {
            return;
        }
        const auto name =
            CompanionLinkFailsafe::parameter_names[failsafe_parameter_index_];
        const bool sent =
            write_frame(hardware::mavlink::encode_parameter_request_read(
                            vehicle.system_id.value(),
                            name),
                now,
                "PARAM_REQUEST_READ",
                std::string(name));
        record_event(LinkEventDirection::outbound,
            sent ? LinkEventStatus::pending : LinkEventStatus::failure,
            "PARAM READ",
            std::string(name) + (sent ? "" : " | WRITE FAILED"),
            now);
        ++failsafe_parameter_index_;
        if (failsafe_parameter_index_ <
            CompanionLinkFailsafe::parameter_names.size()) {
            next_failsafe_parameter_request_ = now + kFailsafeParameterSpacing;
            return;
        }
        failsafe_parameter_index_ = 0;
        next_failsafe_parameter_request_ =
            now + (companion_link_failsafe_.snapshot().accepted()
                          ? kAcceptedFailsafeRefreshInterval
                          : kRejectedFailsafeRetryInterval);
    }

    void request_vehicle_metadata(const mission::VehicleSnapshot& vehicle,
        const bool telemetry_ready,
        const mission::TimePoint now) {
        if (!telemetry_ready || !vehicle.system_id.has_value()) {
            return;
        }
        const auto system_id = vehicle.system_id.value();
        if (!vehicle.autopilot_metadata.has_value() &&
            now >= next_autopilot_version_request_) {
            const bool sent = write_frame(
                hardware::mavlink::encode_autopilot_version_request(system_id),
                now,
                "COMMAND_LONG",
                "REQUEST AUTOPILOT_VERSION");
            next_autopilot_version_request_ =
                now + kAutopilotVersionRetryInterval;
            record_event(LinkEventDirection::outbound,
                sent ? LinkEventStatus::pending : LinkEventStatus::failure,
                "REQUEST_VERSION",
                sent ? "AUTOPILOT_VERSION" : "AUTOPILOT_VERSION | WRITE FAILED",
                now);
        }
        if (vehicle.battery_arming_voltage_v.has_value() ||
            now < next_battery_parameter_request_) {
            return;
        }
        const bool sent = write_frame(
            hardware::mavlink::encode_battery_arming_voltage_request(system_id),
            now,
            "PARAM_REQUEST_READ",
            "BATT_ARM_VOLT");
        next_battery_parameter_request_ = now + kBatteryParameterRetryInterval;
        record_event(LinkEventDirection::outbound,
            sent ? LinkEventStatus::pending : LinkEventStatus::failure,
            "PARAM READ",
            sent ? "BATT_ARM_VOLT" : "BATT_ARM_VOLT | WRITE FAILED",
            now);
    }

    void poll(const std::optional<mission::TimePoint> fixed_now) {
        const auto now = poll_inputs(fixed_now);
        const auto vehicle = vehicle_state_.snapshot(now);
        update_vehicle_connection(vehicle, now);
        send_companion_heartbeat(vehicle, now);
        const bool telemetry_ready = update_telemetry_setup(vehicle, now);
        request_failsafe_parameter(vehicle, telemetry_ready, now);
        request_vehicle_metadata(vehicle, telemetry_ready, now);
    }

  public:
    AppSnapshot snapshot(const mission::TimePoint now) {
        const auto telemetry = telemetry_configurator_.snapshot();
        return {
            .vehicle = vehicle_state_.snapshot(now),
            .companion_heartbeat_active = companion_heartbeat_active_,
            .companion_link_failsafe = companion_link_failsafe_.snapshot(),
            .telemetry =
                {
                    .state = map_telemetry_state(telemetry.phase),
                    .completed_requests = telemetry.completed_requests,
                    .total_requests = telemetry.total_requests,
                    .current_stream = telemetry.current_stream,
                    .attempt = telemetry.attempt,
                    .failure_result = telemetry.failure_result,
                },
            .simulated_wind = simulated_wind_,
            .camera = camera_monitor_.has_value()
                          ? std::optional{camera_monitor_->snapshot(now)}
                          : std::nullopt,
            .vision = camera_monitor_.has_value()
                          ? camera_monitor_->vision_snapshot(now)
                          : std::nullopt,
            .link_events =
                {
                    link_events_.begin(),
                    link_events_.end(),
                },
            .elapsed = elapsed_at(now),
            .tx_activity = tx_activity_,
            .rx_activity = rx_activity_,
        };
    }

    [[nodiscard]] std::optional<ProcessedCameraFrame>
    take_latest_processed_camera_frame() {
        if (!camera_monitor_.has_value()) {
            return std::nullopt;
        }
        return camera_monitor_->take_latest_processed_frame();
    }

  private:
    void observe_failsafe_phase(const mission::TimePoint now) {
        const auto current = companion_link_failsafe_.snapshot();
        if (current.phase == previous_failsafe_phase_) {
            return;
        }
        for (auto* sink : observation_sinks_) {
            if (sink != nullptr) {
                sink->consume_failsafe_transition(previous_failsafe_phase_,
                    current, elapsed_at(now), std::chrono::system_clock::now());
            }
        }
        previous_failsafe_phase_ = current.phase;
    }

    void record_event(const LinkEventDirection direction,
        const LinkEventStatus status,
        std::string label,
        std::string detail,
        const mission::TimePoint now) {
        link_events_.push_back({
            .sequence = ++next_event_sequence_,
            .elapsed = elapsed_at(now),
            .direction = direction,
            .status = status,
            .label = std::move(label),
            .detail = std::move(detail),
        });
        for (auto* sink : observation_sinks_) {
            if (sink != nullptr) {
                sink->consume_link_event(link_events_.back(),
                    std::chrono::system_clock::now());
            }
        }
        if (link_events_.size() > kMaximumLinkEvents) {
            link_events_.pop_front();
        }
    }

    std::chrono::milliseconds elapsed_at(const mission::TimePoint now) {
        if (!started_at_.has_value()) {
            started_at_ = now;
        }
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            now - *started_at_);
    }

    void record_activity(const LinkEventDirection direction,
        std::string message_name,
        std::string detail,
        const mission::TimePoint now) {
        LinkActivity activity{
            .sequence = ++next_activity_sequence_,
            .observed_at = elapsed_at(now),
            .message_name = std::move(message_name),
            .detail = std::move(detail),
        };
        if (direction == LinkEventDirection::outbound) {
            tx_activity_ = std::move(activity);
        } else {
            rx_activity_ = std::move(activity);
        }
    }

    void observe_vehicle(const mission::VehicleSnapshot& vehicle,
        const mission::TimePoint now) {
        if (!vehicle.connected) {
            if (vehicle_was_connected_) {
                record_event(LinkEventDirection::inbound,
                    LinkEventStatus::warning,
                    "LINK",
                    "HEARTBEAT LOST",
                    now);
            }
            vehicle_was_connected_ = false;
            previous_flight_mode_.reset();
            previous_armed_.reset();
            autopilot_metadata_was_available_ = false;
            return;
        }

        if (!vehicle_was_connected_) {
            std::string detail{"FLIGHT CONTROLLER ONLINE"};
            if (vehicle.system_id.has_value()) {
                detail += " | SYS " + std::to_string(*vehicle.system_id);
            }
            if (vehicle.component_id.has_value()) {
                detail += " COMP " + std::to_string(*vehicle.component_id);
            }
            record_event(LinkEventDirection::inbound,
                LinkEventStatus::success,
                "HEARTBEAT",
                std::move(detail),
                now);
            vehicle_was_connected_ = true;
            previous_flight_mode_ = vehicle.flight_mode;
            previous_armed_ = vehicle.armed;
            autopilot_metadata_was_available_ =
                vehicle.autopilot_metadata.has_value();
            return;
        }

        if (vehicle.autopilot_metadata.has_value() &&
            !autopilot_metadata_was_available_) {
            const auto& metadata = *vehicle.autopilot_metadata;
            record_event(LinkEventDirection::inbound,
                LinkEventStatus::success,
                "AUTOPILOT_VERSION",
                std::to_string(metadata.firmware_major) + "." +
                    std::to_string(metadata.firmware_minor) + "." +
                    std::to_string(metadata.firmware_patch),
                now);
        }
        autopilot_metadata_was_available_ =
            vehicle.autopilot_metadata.has_value();

        if (vehicle.flight_mode.has_value() &&
            vehicle.flight_mode != previous_flight_mode_) {
            record_event(LinkEventDirection::inbound,
                LinkEventStatus::success,
                "HEARTBEAT",
                "MODE " + flight_mode_name(*vehicle.flight_mode),
                now);
        }
        previous_flight_mode_ = vehicle.flight_mode;

        if (previous_armed_.has_value() && vehicle.armed != *previous_armed_) {
            record_event(LinkEventDirection::inbound,
                LinkEventStatus::success,
                "HEARTBEAT",
                vehicle.armed ? "ARMED" : "DISARMED",
                now);
        }
        previous_armed_ = vehicle.armed;
    }

    bool write_frame(const std::span<const std::uint8_t> frame,
        const mission::TimePoint now,
        std::string message_name,
        std::string detail = {}) {
        std::size_t offset = 0;
        while (offset < frame.size()) {
            const auto written = transport_.write(frame.subspan(offset));
            if (written == 0) {
                return false;
            }
            offset += written;
        }
        record_activity(LinkEventDirection::outbound,
            std::move(message_name),
            std::move(detail),
            now);
        return true;
    }

    static constexpr auto kHeartbeatInterval = std::chrono::seconds(1);
    static constexpr auto kBatteryParameterRetryInterval =
        std::chrono::seconds(2);
    static constexpr auto kFailsafeParameterSpacing =
        std::chrono::milliseconds(200);
    static constexpr auto kRejectedFailsafeRetryInterval =
        std::chrono::seconds(2);
    static constexpr auto kAcceptedFailsafeRefreshInterval =
        std::chrono::seconds(10);
    static constexpr auto kAutopilotVersionRetryInterval =
        std::chrono::seconds(2);
    static constexpr std::size_t kMaximumLinkEvents = 8;

    ports::Transport& transport_;
    std::optional<SimulatedWindProfile> simulated_wind_;
    mission::VehicleState vehicle_state_;
    CompanionLinkFailsafe companion_link_failsafe_;
    hardware::mavlink::TelemetryStreamConfigurator telemetry_configurator_;
    std::optional<CameraMonitor> camera_monitor_;
    hardware::mavlink::MavlinkDecoder decoder_;
    std::array<std::uint8_t, kTransportReceiveBufferSize> receive_buffer_{};
    mission::TimePoint next_heartbeat_{};
    mission::TimePoint next_battery_parameter_request_{};
    mission::TimePoint next_failsafe_parameter_request_{};
    mission::TimePoint next_autopilot_version_request_{};
    std::optional<mission::TimePoint> started_at_;
    std::deque<LinkEvent> link_events_;
    std::vector<ports::RuntimeSnapshotSink*> observation_sinks_;
    CompanionLinkFailsafePhase previous_failsafe_phase_{
        CompanionLinkFailsafePhase::waiting_for_vehicle};
    std::uint64_t next_event_sequence_{0};
    std::uint64_t next_activity_sequence_{0};
    std::optional<LinkActivity> tx_activity_;
    std::optional<LinkActivity> rx_activity_;
    bool vehicle_was_connected_{false};
    std::optional<std::uint32_t> previous_flight_mode_;
    std::optional<bool> previous_armed_;
    bool autopilot_metadata_was_available_{false};
    bool companion_heartbeat_active_{false};
    std::size_t failsafe_parameter_index_{0};
};

CompanionApplication::CompanionApplication(ports::Transport& transport,
    CompanionApplicationOptions options)
    : impl_(std::make_unique<Impl>(transport, options)) {}

CompanionApplication::~CompanionApplication() = default;

void CompanionApplication::poll(const mission::TimePoint now) {
    impl_->poll(now);
}

void CompanionApplication::poll() { impl_->poll(); }

void CompanionApplication::set_observation_sinks(
    std::vector<ports::RuntimeSnapshotSink*> sinks) {
    impl_->set_observation_sinks(std::move(sinks));
}

AppSnapshot CompanionApplication::snapshot(const mission::TimePoint now) {
    return impl_->snapshot(now);
}

std::optional<ProcessedCameraFrame>
CompanionApplication::take_latest_processed_camera_frame() {
    return impl_->take_latest_processed_camera_frame();
}

} // namespace onboard_autonomy::mission
