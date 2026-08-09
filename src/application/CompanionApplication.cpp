#include "onboard_autonomy/application/CompanionApplication.hpp"

#include "onboard_autonomy/adapters/mavlink/MavlinkDecoder.hpp"
#include "onboard_autonomy/adapters/mavlink/MavlinkEncoder.hpp"
#include "onboard_autonomy/adapters/mavlink/TelemetryStreamConfigurator.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <optional>
#include <sstream>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace onboard_autonomy::application {
namespace {

TelemetrySetupState map_telemetry_state(
    const adapters::mavlink::TelemetrySetupPhase phase
) {
    switch (phase) {
        case adapters::mavlink::TelemetrySetupPhase::waiting_for_vehicle:
            return TelemetrySetupState::waiting_for_vehicle;
        case adapters::mavlink::TelemetrySetupPhase::configuring:
            return TelemetrySetupState::configuring;
        case adapters::mavlink::TelemetrySetupPhase::active:
            return TelemetrySetupState::active;
        case adapters::mavlink::TelemetrySetupPhase::failed:
            return TelemetrySetupState::failed;
    }
    return TelemetrySetupState::failed;
}

std::optional<FlightAction> map_flight_action(
    const std::uint16_t command
) {
    switch (command) {
        case MAV_CMD_DO_SET_MODE:
            return FlightAction::set_guided_mode;
        case MAV_CMD_COMPONENT_ARM_DISARM:
            return FlightAction::arm;
        case MAV_CMD_NAV_TAKEOFF:
            return FlightAction::takeoff;
        case MAV_CMD_NAV_LAND:
            return FlightAction::land;
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            return FlightAction::return_to_launch;
        default:
            return std::nullopt;
    }
}

FlightCommandAckOutcome map_ack_outcome(const std::uint8_t result) {
    if (result == MAV_RESULT_ACCEPTED) {
        return FlightCommandAckOutcome::accepted;
    }
    if (result == MAV_RESULT_IN_PROGRESS) {
        return FlightCommandAckOutcome::in_progress;
    }
    return FlightCommandAckOutcome::rejected;
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
        default:
            return "ACK COMMAND " + std::to_string(command);
    }
}

std::string command_ack_activity_detail(
    const std::uint16_t command,
    const std::uint8_t result
) {
    std::string command_name = mav_command_name(command);
    if (command_name.starts_with("ACK ")) {
        command_name.erase(0, 4);
    }
    return command_name + " " + ack_result_name(result);
}

std::string flight_action_name(const FlightAction action) {
    switch (action) {
        case FlightAction::set_guided_mode:
            return "SET_MODE";
        case FlightAction::arm:
            return "ARM";
        case FlightAction::takeoff:
            return "TAKEOFF";
        case FlightAction::return_to_launch:
            return "RTL";
        case FlightAction::land:
            return "LAND";
        case FlightAction::landing_target:
            return "LANDING_TARGET";
    }
    return "ACTION";
}

std::string flight_action_message_name(const FlightAction action) {
    switch (action) {
        case FlightAction::landing_target:
            return "LANDING_TARGET";
        case FlightAction::set_guided_mode:
        case FlightAction::arm:
        case FlightAction::takeoff:
        case FlightAction::return_to_launch:
        case FlightAction::land:
            return "COMMAND_LONG";
    }
    return "MAVLINK";
}

bool action_expects_ack(const FlightAction action) {
    return action != FlightAction::landing_target;
}

std::string flight_action_detail(
    const FlightActionRequest& request
) {
    std::string detail;
    switch (request.action) {
        case FlightAction::set_guided_mode:
            detail = "GUIDED";
            break;
        case FlightAction::arm:
            detail = "SAFETY CHECKS ON";
            break;
        case FlightAction::takeoff: {
            std::ostringstream altitude;
            altitude << std::fixed << std::setprecision(1)
                     << request.altitude_m << " M";
            detail = altitude.str();
            break;
        }
        case FlightAction::return_to_launch:
            detail = "RETURN TO HOME";
            break;
        case FlightAction::land:
            detail = "LAND MODE";
            break;
        case FlightAction::landing_target: {
            std::ostringstream target;
            target << std::fixed << std::setprecision(1)
                   << "F/R/D " << request.x_m << "/"
                   << request.y_m << "/" << request.z_m << " M";
            return target.str();
        }
    }

    if (action_expects_ack(request.action)) {
        detail += " | ATTEMPT " +
                  std::to_string(
                      static_cast<unsigned int>(
                          request.confirmation
                      ) +
                      1
                  );
    }
    return detail;
}

std::string flight_mode_name(const std::uint32_t mode) {
    switch (mode) {
        case 0:
            return "STABILIZE";
        case 3:
            return "AUTO";
        case 4:
            return "GUIDED";
        case 5:
            return "LOITER";
        case 6:
            return "RTL";
        case 9:
            return "LAND";
        case 16:
            return "POSITION HOLD";
        default:
            return "MODE " + std::to_string(mode);
    }
}

std::vector<std::uint8_t> encode_flight_action(
    const FlightActionRequest& request
) {
    switch (request.action) {
        case FlightAction::set_guided_mode:
            return adapters::mavlink::encode_set_guided_mode(
                request.vehicle_system_id,
                request.confirmation
            );
        case FlightAction::arm:
            return adapters::mavlink::encode_arm(
                request.vehicle_system_id,
                request.confirmation
            );
        case FlightAction::takeoff:
            return adapters::mavlink::encode_takeoff(
                request.vehicle_system_id,
                request.altitude_m,
                request.confirmation
            );
        case FlightAction::return_to_launch:
            return adapters::mavlink::encode_return_to_launch(
                request.vehicle_system_id,
                request.confirmation
            );
        case FlightAction::land:
            return adapters::mavlink::encode_land(
                request.vehicle_system_id,
                request.confirmation
            );
        case FlightAction::landing_target:
            return adapters::mavlink::encode_landing_target(
                request.vehicle_system_id,
                request.time_usec,
                request.x_m,
                request.y_m,
                request.z_m
            );
    }
    return {};
}

}  // namespace

class CompanionApplication::Impl {
public:
    explicit Impl(
        ports::Transport& transport,
        CompanionApplicationOptions options
    )
        : transport_(transport),
          motion_commands_allowed_(
              options.motion_commands_allowed
          ),
          autonomy_scenario_configured_(
              options.flight_startup.enabled &&
              options.autonomy_runtime.enabled
          ),
          simulated_wind_(options.simulated_wind),
          flight_startup_(options.flight_startup),
          autonomy_runtime_(options.autonomy_runtime),
          camera_extrinsics_(options.camera_extrinsics),
          decoder_{
              vehicle_state_,
              [this](
                  const adapters::mavlink::CommandAck& acknowledgement,
                  const domain::TimePoint now
              ) {
                  const auto telemetry_before =
                      telemetry_configurator_.snapshot();

                  if (acknowledgement.target_component == 0 ||
                      acknowledgement.target_component ==
                          adapters::mavlink::kCompanionComponentId) {
                      std::string detail =
                          ack_result_name(acknowledgement.result);
                      if (acknowledgement.command ==
                              MAV_CMD_SET_MESSAGE_INTERVAL &&
                          telemetry_before.phase ==
                              adapters::mavlink::
                                  TelemetrySetupPhase::configuring &&
                          !telemetry_before.current_stream.empty()) {
                          detail =
                              telemetry_before.current_stream +
                              " | " + detail;
                      }
                      detail +=
                          " | SYS " +
                          std::to_string(
                              acknowledgement.source_system
                          );
                      record_event(
                          LinkEventDirection::inbound,
                          map_event_status(
                              acknowledgement.result
                          ),
                          mav_command_name(
                              acknowledgement.command
                          ),
                          std::move(detail),
                          now
                      );
                      record_activity(
                          LinkEventDirection::inbound,
                          "COMMAND_ACK",
                          command_ack_activity_detail(
                              acknowledgement.command,
                              acknowledgement.result
                          ),
                          now
                      );
                  }

                  telemetry_configurator_.on_command_ack(
                      acknowledgement,
                      now
                  );

                  const auto flight_action =
                      map_flight_action(acknowledgement.command);
                  if (!flight_action.has_value() ||
                      (acknowledgement.target_component != 0 &&
                       acknowledgement.target_component !=
                           adapters::mavlink::
                               kCompanionComponentId)) {
                      return;
                  }

                  const auto outcome =
                      map_ack_outcome(acknowledgement.result);
                  flight_startup_.on_command_ack(
                      *flight_action,
                      outcome,
                      acknowledgement.result,
                      acknowledgement.source_system,
                      now
                  );
                  autonomy_runtime_.on_command_ack(
                      *flight_action,
                      outcome,
                      acknowledgement.result,
                      acknowledgement.source_system,
                      now
                  );
              },
              [this](
                  const adapters::mavlink::MessageObservation& message,
                  const domain::TimePoint now
              ) {
                  record_activity(
                      LinkEventDirection::inbound,
                      message.message_name.empty()
                          ? "MSG #" +
                                std::to_string(message.message_id)
                          : std::string(message.message_name),
                      "",
                      now
                  );
              },
              [this](
                  const adapters::mavlink::ParameterValue& parameter,
                  const domain::TimePoint now
              ) {
                  companion_link_failsafe_.on_parameter(
                      parameter.source_system,
                      parameter.source_component,
                      parameter.id,
                      parameter.value
                  );
                  for (const auto expected :
                       CompanionLinkFailsafe::parameter_names) {
                      if (parameter.id == expected) {
                          record_activity(
                              LinkEventDirection::inbound,
                              "PARAM_VALUE",
                              parameter.id,
                              now
                          );
                          break;
                      }
                  }
              },
          } {
        const auto startup = flight_startup_.snapshot();
        const auto runtime = autonomy_runtime_.snapshot();
        const bool startup_enabled =
            startup.phase != FlightStartupPhase::disabled;
        const bool runtime_enabled =
            runtime.phase != AutonomyRuntimePhase::disabled;
        if (startup_enabled != runtime_enabled) {
            throw std::invalid_argument(
                "flight startup and autonomy runtime must be enabled "
                "together"
            );
        }
        if (!motion_commands_allowed_ && startup_enabled) {
            throw std::invalid_argument(
                "automated flight requires explicit motion permission"
            );
        }
        if (options.camera_source != nullptr) {
            camera_monitor_.emplace(
                *options.camera_source,
                options.target_detector,
                options.camera_preview_sink
            );
        } else if (options.target_detector != nullptr ||
                   options.camera_preview_sink != nullptr) {
            throw std::invalid_argument(
                "vision and preview require a camera source"
            );
        }
        if (camera_extrinsics_.has_value() &&
            options.target_detector == nullptr) {
            throw std::invalid_argument(
                "camera extrinsics require AprilTag pose detection"
            );
        }
        if (runtime_enabled && !camera_extrinsics_.has_value()) {
            throw std::invalid_argument(
                "autonomy runtime requires vision guidance"
            );
        }
    }

    void poll() {
        poll(std::nullopt);
    }

    void poll(const domain::TimePoint now) {
        poll(std::optional{now});
    }

private:
    void poll(const std::optional<domain::TimePoint> fixed_now) {
        const auto camera_now = fixed_now.value_or(
            domain::Clock::now()
        );
        if (!started_at_.has_value()) {
            started_at_ = camera_now;
        }
        if (camera_monitor_.has_value()) {
            camera_monitor_->poll(camera_now);
        }

        const std::size_t received = transport_.read(receive_buffer_);
        const auto now = fixed_now.value_or(domain::Clock::now());
        if (received > 0) {
            decoder_.ingest(
                std::span<const std::uint8_t>{
                    receive_buffer_.data(),
                    received,
                },
                now
            );
        }

        const auto vehicle = vehicle_state_.snapshot(now);
        const bool newly_connected =
            vehicle.connected && !vehicle_was_connected_;
        companion_link_failsafe_.observe_vehicle(
            vehicle.connected,
            vehicle.system_id
        );
        if (newly_connected) {
            failsafe_parameter_index_ = 0;
            next_failsafe_parameter_request_ = now;
        }
        observe_vehicle(vehicle, now);
        if (!vehicle.connected) {
            companion_heartbeat_active_ = false;
        }

        if (now >= next_heartbeat_) {
            if (vehicle.connected && vehicle.system_id.has_value()) {
                const auto heartbeat =
                    adapters::mavlink::encode_companion_heartbeat(
                        *vehicle.system_id
                    );
                if (write_frame(
                        heartbeat,
                        now,
                        "HEARTBEAT"
                    )) {
                    companion_heartbeat_active_ = true;
                }
            }
            next_heartbeat_ = now + kHeartbeatInterval;
        }

        const auto telemetry_frame = telemetry_configurator_.update(
            vehicle.connected,
            vehicle.system_id,
            now
        );
        if (telemetry_frame.has_value()) {
            const auto setup = telemetry_configurator_.snapshot();
            const bool sent = write_frame(
                *telemetry_frame,
                now,
                "COMMAND_LONG",
                "SET_INTERVAL " + setup.current_stream
            );
            record_event(
                LinkEventDirection::outbound,
                sent
                    ? LinkEventStatus::pending
                    : LinkEventStatus::failure,
                "SET_INTERVAL",
                setup.current_stream + " | ATTEMPT " +
                    std::to_string(setup.attempt) +
                    (sent ? "" : " | WRITE FAILED"),
                now
            );
        }

        const bool telemetry_ready =
            telemetry_configurator_.snapshot().phase ==
            adapters::mavlink::TelemetrySetupPhase::active;
        if (telemetry_ready &&
            vehicle.system_id.has_value() &&
            now >= next_failsafe_parameter_request_) {
            const auto parameter_name =
                CompanionLinkFailsafe::parameter_names[
                    failsafe_parameter_index_
                ];
            const bool sent = write_frame(
                adapters::mavlink::encode_parameter_request_read(
                    *vehicle.system_id,
                    parameter_name
                ),
                now,
                "PARAM_REQUEST_READ",
                std::string(parameter_name)
            );
            record_event(
                LinkEventDirection::outbound,
                sent
                    ? LinkEventStatus::pending
                    : LinkEventStatus::failure,
                "PARAM READ",
                std::string(parameter_name) +
                    (sent ? "" : " | WRITE FAILED"),
                now
            );

            ++failsafe_parameter_index_;
            if (failsafe_parameter_index_ >=
                CompanionLinkFailsafe::parameter_names.size()) {
                failsafe_parameter_index_ = 0;
                next_failsafe_parameter_request_ =
                    now +
                    (companion_link_failsafe_.snapshot().accepted()
                         ? kAcceptedFailsafeRefreshInterval
                         : kRejectedFailsafeRetryInterval);
            } else {
                next_failsafe_parameter_request_ =
                    now + kFailsafeParameterSpacing;
            }
        }
        if (telemetry_ready &&
            vehicle.system_id.has_value() &&
            !vehicle.autopilot_metadata.has_value() &&
            now >= next_autopilot_version_request_) {
            const bool sent = write_frame(
                adapters::mavlink::encode_autopilot_version_request(
                    *vehicle.system_id
                ),
                now,
                "COMMAND_LONG",
                "REQUEST AUTOPILOT_VERSION"
            );
            next_autopilot_version_request_ =
                now + kAutopilotVersionRetryInterval;
            record_event(
                LinkEventDirection::outbound,
                sent
                    ? LinkEventStatus::pending
                    : LinkEventStatus::failure,
                "REQUEST_VERSION",
                sent
                    ? "AUTOPILOT_VERSION"
                    : "AUTOPILOT_VERSION | WRITE FAILED",
                now
            );
        }
        if (telemetry_ready &&
            vehicle.system_id.has_value() &&
            !vehicle.battery_arming_voltage_v.has_value() &&
            now >= next_battery_parameter_request_) {
            const bool sent = write_frame(
                adapters::mavlink::
                    encode_battery_arming_voltage_request(
                        *vehicle.system_id
                    ),
                now,
                "PARAM_REQUEST_READ",
                "BATT_ARM_VOLT"
            );
            next_battery_parameter_request_ =
                now + kBatteryParameterRetryInterval;
            record_event(
                LinkEventDirection::outbound,
                sent
                    ? LinkEventStatus::pending
                    : LinkEventStatus::failure,
                "PARAM READ",
                sent
                    ? "BATT_ARM_VOLT"
                    : "BATT_ARM_VOLT | WRITE FAILED",
                now
            );
        }

        const auto startup_actions = flight_startup_.update(
            vehicle,
            telemetry_ready,
            companion_link_failsafe_.snapshot(),
            now
        );
        for (const auto& action : startup_actions) {
            const bool sent = write_frame(
                encode_flight_action(action),
                now,
                flight_action_message_name(action.action),
                flight_action_name(action.action)
            );
            flight_startup_.on_action_sent(action, sent, now);
            record_event(
                LinkEventDirection::outbound,
                sent
                    ? LinkEventStatus::pending
                    : LinkEventStatus::failure,
                flight_action_name(action.action),
                flight_action_detail(action) +
                    (sent ? "" : " | WRITE FAILED"),
                now
            );
        }

        const auto autonomy_actions = autonomy_runtime_.update(
            vehicle,
            flight_startup_.snapshot(),
            companion_link_failsafe_.snapshot(),
            now,
            current_landing_target(now)
        );
        for (const auto& action : autonomy_actions) {
            const bool sent = write_frame(
                encode_flight_action(action),
                now,
                flight_action_message_name(action.action),
                flight_action_name(action.action)
            );
            autonomy_runtime_.on_action_sent(action, sent, now);
            record_event(
                LinkEventDirection::outbound,
                sent
                    ? LinkEventStatus::pending
                    : LinkEventStatus::failure,
                flight_action_name(action.action),
                flight_action_detail(action) +
                    (sent ? "" : " | WRITE FAILED"),
                now
            );
        }
    }

public:
    bool request_autonomy_start(const domain::TimePoint now) {
        if (!motion_commands_allowed_) {
            record_event(
                LinkEventDirection::outbound,
                LinkEventStatus::failure,
                "START",
                "BLOCKED BY MOTION SAFETY POLICY",
                now
            );
            return false;
        }

        if (!autonomy_scenario_configured_) {
            record_event(
                LinkEventDirection::outbound,
                LinkEventStatus::failure,
                "START",
                "AUTONOMOUS SCENARIO NOT CONFIGURED",
                now
            );
            return false;
        }

        const auto vehicle = vehicle_state_.snapshot(now);
        if (!vehicle.connected || !vehicle.system_id.has_value()) {
            record_event(
                LinkEventDirection::outbound,
                LinkEventStatus::failure,
                "START",
                "BLOCKED | NO FLIGHT CONTROLLER",
                now
            );
            return false;
        }

        if (vehicle.armed) {
            record_event(
                LinkEventDirection::outbound,
                LinkEventStatus::failure,
                "START",
                "BLOCKED | VEHICLE IS ARMED",
                now
            );
            return false;
        }

        const auto startup = flight_startup_.snapshot();
        const auto autonomy = autonomy_runtime_.snapshot();
        const bool startup_finished =
            startup.phase == FlightStartupPhase::completed ||
            startup.phase == FlightStartupPhase::failed;
        const bool autonomy_finished =
            autonomy.phase == AutonomyRuntimePhase::completed ||
            autonomy.phase == AutonomyRuntimePhase::failed;
        if (!startup_finished || !autonomy_finished) {
            record_event(
                LinkEventDirection::outbound,
                LinkEventStatus::warning,
                "START",
                "SCENARIO ALREADY RUNNING",
                now
            );
            return false;
        }

        flight_startup_.restart();
        autonomy_runtime_.restart();
        record_event(
            LinkEventDirection::outbound,
            LinkEventStatus::pending,
            "START",
            "AUTONOMOUS FLIGHT REQUESTED",
            now
        );
        return true;
    }

    AppSnapshot snapshot(const domain::TimePoint now) {
        const auto telemetry = telemetry_configurator_.snapshot();
        return {
            .vehicle = vehicle_state_.snapshot(now),
            .companion_heartbeat_active =
                companion_heartbeat_active_,
            .companion_link_failsafe =
                companion_link_failsafe_.snapshot(),
            .telemetry =
                {
                    .state = map_telemetry_state(telemetry.phase),
                    .completed_requests =
                        telemetry.completed_requests,
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
            .flight_startup = flight_startup_.snapshot(),
            .autonomy = autonomy_runtime_.snapshot(),
            .motion_commands_allowed = motion_commands_allowed_,
            .link_events = {
                link_events_.begin(),
                link_events_.end(),
            },
            .elapsed = elapsed_at(now),
            .tx_activity = tx_activity_,
            .rx_activity = rx_activity_,
        };
    }

private:
    [[nodiscard]] std::optional<domain::BodyFramePosition>
    current_landing_target(const domain::TimePoint now) const {
        if (!camera_monitor_.has_value() ||
            !camera_extrinsics_.has_value()) {
            return std::nullopt;
        }

        const auto vision = camera_monitor_->vision_snapshot(now);
        if (!vision.has_value() ||
            vision->target_track.phase != TargetTrackPhase::tracking ||
            !vision->target_track.position.has_value() ||
            !vision->target_track.observation_age_ms.has_value() ||
            *vision->target_track.observation_age_ms >
                kMaximumLandingTargetAge.count()) {
            return std::nullopt;
        }

        return domain::camera_to_body_frd(
            *vision->target_track.position,
            *camera_extrinsics_
        );
    }

    void record_event(
        const LinkEventDirection direction,
        const LinkEventStatus status,
        std::string label,
        std::string detail,
        const domain::TimePoint now
    ) {
        link_events_.push_back(
            {
                .sequence = ++next_event_sequence_,
                .elapsed = elapsed_at(now),
                .direction = direction,
                .status = status,
                .label = std::move(label),
                .detail = std::move(detail),
            }
        );
        if (link_events_.size() > kMaximumLinkEvents) {
            link_events_.pop_front();
        }
    }

    std::chrono::milliseconds elapsed_at(
        const domain::TimePoint now
    ) {
        if (!started_at_.has_value()) {
            started_at_ = now;
        }
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            now - *started_at_
        );
    }

    void record_activity(
        const LinkEventDirection direction,
        std::string message_name,
        std::string detail,
        const domain::TimePoint now
    ) {
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

    void observe_vehicle(
        const domain::VehicleSnapshot& vehicle,
        const domain::TimePoint now
    ) {
        if (!vehicle.connected) {
            if (vehicle_was_connected_) {
                record_event(
                    LinkEventDirection::inbound,
                    LinkEventStatus::warning,
                    "LINK",
                    "HEARTBEAT LOST",
                    now
                );
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
                detail +=
                    " | SYS " +
                    std::to_string(*vehicle.system_id);
            }
            if (vehicle.component_id.has_value()) {
                detail +=
                    " COMP " +
                    std::to_string(*vehicle.component_id);
            }
            record_event(
                LinkEventDirection::inbound,
                LinkEventStatus::success,
                "HEARTBEAT",
                std::move(detail),
                now
            );
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
            record_event(
                LinkEventDirection::inbound,
                LinkEventStatus::success,
                "AUTOPILOT_VERSION",
                std::to_string(metadata.firmware_major) + "." +
                    std::to_string(metadata.firmware_minor) + "." +
                    std::to_string(metadata.firmware_patch),
                now
            );
        }
        autopilot_metadata_was_available_ =
            vehicle.autopilot_metadata.has_value();

        if (vehicle.flight_mode.has_value() &&
            vehicle.flight_mode != previous_flight_mode_) {
            record_event(
                LinkEventDirection::inbound,
                LinkEventStatus::success,
                "HEARTBEAT",
                "MODE " +
                    flight_mode_name(*vehicle.flight_mode),
                now
            );
        }
        previous_flight_mode_ = vehicle.flight_mode;

        if (previous_armed_.has_value() &&
            vehicle.armed != *previous_armed_) {
            record_event(
                LinkEventDirection::inbound,
                LinkEventStatus::success,
                "HEARTBEAT",
                vehicle.armed ? "ARMED" : "DISARMED",
                now
            );
        }
        previous_armed_ = vehicle.armed;
    }

    bool write_frame(
        const std::span<const std::uint8_t> frame,
        const domain::TimePoint now,
        std::string message_name,
        std::string detail = {}
    ) {
        std::size_t offset = 0;
        while (offset < frame.size()) {
            const auto written = transport_.write(frame.subspan(offset));
            if (written == 0) {
                return false;
            }
            offset += written;
        }
        record_activity(
            LinkEventDirection::outbound,
            std::move(message_name),
            std::move(detail),
            now
        );
        return true;
    }

    static constexpr auto kHeartbeatInterval =
        std::chrono::seconds(1);
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
    static constexpr auto kMaximumLandingTargetAge =
        std::chrono::milliseconds(250);
    static constexpr std::size_t kMaximumLinkEvents = 8;

    ports::Transport& transport_;
    bool motion_commands_allowed_{false};
    bool autonomy_scenario_configured_{false};
    std::optional<SimulatedWindProfile> simulated_wind_;
    domain::VehicleState vehicle_state_;
    CompanionLinkFailsafe companion_link_failsafe_;
    adapters::mavlink::TelemetryStreamConfigurator telemetry_configurator_;
    FlightStartupController flight_startup_;
    AutonomyRuntime autonomy_runtime_;
    std::optional<CameraMonitor> camera_monitor_;
    std::optional<domain::CameraExtrinsics> camera_extrinsics_;
    adapters::mavlink::MavlinkDecoder decoder_;
    std::array<std::uint8_t, 4096> receive_buffer_{};
    domain::TimePoint next_heartbeat_{};
    domain::TimePoint next_battery_parameter_request_{};
    domain::TimePoint next_failsafe_parameter_request_{};
    domain::TimePoint next_autopilot_version_request_{};
    std::optional<domain::TimePoint> started_at_;
    std::deque<LinkEvent> link_events_;
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

CompanionApplication::CompanionApplication(
    ports::Transport& transport,
    CompanionApplicationOptions options
)
    : impl_(std::make_unique<Impl>(
          transport,
          std::move(options)
      )) {}

CompanionApplication::~CompanionApplication() = default;

void CompanionApplication::poll(const domain::TimePoint now) {
    impl_->poll(now);
}

void CompanionApplication::poll() {
    impl_->poll();
}

bool CompanionApplication::request_autonomy_start(
    const domain::TimePoint now
) {
    return impl_->request_autonomy_start(now);
}

AppSnapshot CompanionApplication::snapshot(const domain::TimePoint now) {
    return impl_->snapshot(now);
}

}  // namespace onboard_autonomy::application
