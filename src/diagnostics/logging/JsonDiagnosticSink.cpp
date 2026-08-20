#include "onboard_autonomy/diagnostics/logging/JsonDiagnosticSink.hpp"

#include "onboard_autonomy/application/AppSnapshot.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace onboard_autonomy::diagnostics::logging {
namespace {

using Json = nlohmann::json;

template <typename T> Json optional_number(const std::optional<T>& value) {
    return value.has_value() ? Json(*value) : Json(nullptr);
}

Json optional_byte(const std::optional<std::uint8_t>& value) {
    return value.has_value() ? Json(static_cast<unsigned int>(*value))
                             : Json(nullptr);
}

Json optional_signed_byte(const std::optional<std::int8_t>& value) {
    return value.has_value() ? Json(static_cast<int>(*value)) : Json(nullptr);
}

std::string_view camera_phase_name(
    const application::ports::CameraSourcePhase phase) {
    using application::ports::CameraSourcePhase;
    switch (phase) {
    case CameraSourcePhase::starting:
        return "starting";
    case CameraSourcePhase::streaming:
        return "streaming";
    case CameraSourcePhase::reconnecting:
        return "reconnecting";
    case CameraSourcePhase::stopped:
        return "stopped";
    case CameraSourcePhase::failed:
        return "failed";
    }
    return "failed";
}

std::string_view target_track_phase_name(
    const application::TargetTrackPhase phase) {
    using application::TargetTrackPhase;
    switch (phase) {
    case TargetTrackPhase::searching:
        return "searching";
    case TargetTrackPhase::acquiring:
        return "acquiring";
    case TargetTrackPhase::tracking:
        return "tracking";
    }
    return "searching";
}

std::string_view telemetry_state_name(
    const application::TelemetrySetupState state) {
    using application::TelemetrySetupState;
    switch (state) {
    case TelemetrySetupState::waiting_for_vehicle:
        return "waiting_for_vehicle";
    case TelemetrySetupState::configuring:
        return "configuring";
    case TelemetrySetupState::active:
        return "active";
    case TelemetrySetupState::failed:
        return "failed";
    }
    return "failed";
}

std::string_view startup_phase_name(
    const application::FlightStartupPhase phase) {
    using application::FlightStartupPhase;
    switch (phase) {
    case FlightStartupPhase::disabled:
        return "disabled";
    case FlightStartupPhase::waiting_for_vehicle:
        return "waiting_for_vehicle";
    case FlightStartupPhase::waiting_for_readiness:
        return "waiting_for_readiness";
    case FlightStartupPhase::setting_guided:
        return "setting_guided";
    case FlightStartupPhase::arming:
        return "arming";
    case FlightStartupPhase::taking_off:
        return "taking_off";
    case FlightStartupPhase::completed:
        return "completed";
    case FlightStartupPhase::failed:
        return "failed";
    }
    return "failed";
}

std::string_view autonomy_phase_name(
    const application::AutonomyRuntimePhase phase) {
    using application::AutonomyRuntimePhase;
    switch (phase) {
    case AutonomyRuntimePhase::disabled:
        return "disabled";
    case AutonomyRuntimePhase::waiting_for_startup:
        return "waiting_for_startup";
    case AutonomyRuntimePhase::active:
        return "active";
    case AutonomyRuntimePhase::landing:
        return "landing";
    case AutonomyRuntimePhase::completed:
        return "completed";
    case AutonomyRuntimePhase::failed:
        return "failed";
    }
    return "failed";
}

std::string_view link_direction_name(
    const application::LinkEventDirection direction) {
    return direction == application::LinkEventDirection::outbound ? "outbound"
                                                                  : "inbound";
}

std::string_view link_status_name(const application::LinkEventStatus status) {
    using application::LinkEventStatus;
    switch (status) {
    case LinkEventStatus::neutral:
        return "neutral";
    case LinkEventStatus::pending:
        return "pending";
    case LinkEventStatus::success:
        return "success";
    case LinkEventStatus::warning:
        return "warning";
    case LinkEventStatus::failure:
        return "failure";
    }
    return "failure";
}

Json vehicle_json(const domain::VehicleSnapshot& vehicle) {
    Json result{
        {"connected", vehicle.connected},
        {"gps_ready", vehicle.gps_ready},
        {"battery_ready", vehicle.battery_ready},
        {"system_health_known", vehicle.system_health_known},
        {"system_health_ok", vehicle.system_health_ok},
        {"armable", vehicle.armable},
        {"armed", vehicle.armed},
        {"system_id", optional_byte(vehicle.system_id)},
        {"component_id", optional_byte(vehicle.component_id)},
        {"vehicle_type", optional_byte(vehicle.vehicle_type)},
        {"autopilot_type", optional_byte(vehicle.autopilot_type)},
        {"system_status", optional_byte(vehicle.system_status)},
        {"flight_mode", optional_number(vehicle.flight_mode)},
        {"gps_fix_type", optional_byte(vehicle.gps_fix_type)},
        {"satellites_visible", optional_byte(vehicle.satellites_visible)},
        {"relative_altitude_m", optional_number(vehicle.relative_altitude_m)},
        {"local_north_m", optional_number(vehicle.local_north_m)},
        {"local_east_m", optional_number(vehicle.local_east_m)},
        {"local_down_m", optional_number(vehicle.local_down_m)},
        {"roll_rad", optional_number(vehicle.roll_rad)},
        {"pitch_rad", optional_number(vehicle.pitch_rad)},
        {"yaw_rad", optional_number(vehicle.yaw_rad)},
        {"battery_voltage_v", optional_number(vehicle.battery_voltage_v)},
        {"battery_current_a", optional_number(vehicle.battery_current_a)},
        {"battery_remaining_pct",
            optional_signed_byte(vehicle.battery_remaining_pct)},
        {"battery_arming_voltage_v",
            optional_number(vehicle.battery_arming_voltage_v)},
        {"warnings", vehicle.warnings},
    };

    const auto& metadata = vehicle.autopilot_metadata;
    result["firmware_major"] =
        metadata.has_value() ? Json(metadata->firmware_major) : Json(nullptr);
    result["firmware_minor"] =
        metadata.has_value() ? Json(metadata->firmware_minor) : Json(nullptr);
    result["firmware_patch"] =
        metadata.has_value() ? Json(metadata->firmware_patch) : Json(nullptr);
    result["firmware_release_type"] =
        metadata.has_value() ? Json(metadata->firmware_release_type)
                             : Json(nullptr);
    result["autopilot_capabilities"] =
        metadata.has_value() ? Json(metadata->capabilities) : Json(nullptr);
    result["board_version"] =
        metadata.has_value() ? Json(metadata->board_version) : Json(nullptr);
    result["board_vendor_id"] =
        metadata.has_value() ? Json(metadata->vendor_id) : Json(nullptr);
    result["board_product_id"] =
        metadata.has_value() ? Json(metadata->product_id) : Json(nullptr);
    return result;
}

Json camera_json(const std::optional<application::CameraSnapshot>& camera) {
    if (!camera.has_value()) {
        return nullptr;
    }
    return Json{
        {"phase", camera_phase_name(camera->phase)},
        {"source", camera->source},
        {"error", camera->error},
        {"width", camera->width},
        {"height", camera->height},
        {"received_frames", camera->received_frames},
        {"dropped_before_processing", camera->dropped_before_processing},
        {"camera_restarts", camera->camera_restarts},
        {"frames_with_capture_timestamp",
            camera->frames_with_capture_timestamp},
        {"measured_fps", optional_number(camera->measured_fps)},
        {"latest_latency_ms", optional_number(camera->latest_latency_ms)},
        {"average_latency_ms", optional_number(camera->average_latency_ms)},
        {"maximum_latency_ms", optional_number(camera->maximum_latency_ms)},
        {"latest_frame_age_ms", optional_number(camera->latest_frame_age_ms)},
    };
}

Json target_pose_json(const std::optional<domain::TargetPose>& pose) {
    if (!pose.has_value()) {
        return nullptr;
    }
    return Json{
        {"frame", "camera_optical"},
        {"right_m", pose->position.right_m},
        {"down_m", pose->position.down_m},
        {"forward_m", pose->position.forward_m},
        {"object_space_error", pose->object_space_error},
        {"rotation_tag_to_camera", pose->rotation_tag_to_camera},
    };
}

Json target_json(const domain::TargetObservation& target) {
    return Json{
        {"id", target.id},
        {"family", target.family},
        {"center_x_px", target.center.x_px},
        {"center_y_px", target.center.y_px},
        {"corrected_bits", target.corrected_bits},
        {"decision_margin", target.decision_margin},
        {"pose", target_pose_json(target.pose)},
    };
}

Json track_json(const application::TargetTrackSnapshot& track) {
    Json position = nullptr;
    if (track.position.has_value()) {
        position = {
            {"frame", "camera_optical"},
            {"right_m", track.position->right_m},
            {"down_m", track.position->down_m},
            {"forward_m", track.position->forward_m},
        };
    }
    return Json{
        {"phase", target_track_phase_name(track.phase)},
        {"target_id", optional_number(track.target_id)},
        {"consecutive_observations", track.consecutive_observations},
        {"required_observations", track.required_observations},
        {"accepted_observations", track.accepted_observations},
        {"observation_age_ms", optional_number(track.observation_age_ms)},
        {"latest_decision_margin",
            optional_number(track.latest_decision_margin)},
        {"position", std::move(position)},
    };
}

Json vision_json(const std::optional<application::VisionSnapshot>& vision) {
    if (!vision.has_value()) {
        return nullptr;
    }
    Json targets = Json::array();
    for (const auto& target : vision->latest_targets) {
        targets.push_back(target_json(target));
    }
    return Json{
        {"detector", vision->detector},
        {"processed_frames", vision->processed_frames},
        {"frames_with_targets", vision->frames_with_targets},
        {"total_targets", vision->total_targets},
        {"latest_processing_ms", optional_number(vision->latest_processing_ms)},
        {"average_processing_ms",
            optional_number(vision->average_processing_ms)},
        {"maximum_processing_ms",
            optional_number(vision->maximum_processing_ms)},
        {"last_detection_age_ms",
            optional_number(vision->last_detection_age_ms)},
        {"targets", std::move(targets)},
        {"target_track", track_json(vision->target_track)},
    };
}

Json link_event_json(const application::LinkEvent& event) {
    return Json{
        {"sequence", event.sequence},
        {"elapsed_ms", event.elapsed.count()},
        {"direction", link_direction_name(event.direction)},
        {"status", link_status_name(event.status)},
        {"label", event.label},
        {"detail", event.detail},
    };
}

Json link_activity_json(
    const std::optional<application::LinkActivity>& activity) {
    if (!activity.has_value()) {
        return nullptr;
    }
    return Json{
        {"sequence", activity->sequence},
        {"observed_at_ms", activity->observed_at.count()},
        {"message_name", activity->message_name},
        {"detail", activity->detail},
    };
}

Json snapshot_json(const application::AppSnapshot& snapshot,
    const std::int64_t recorded_at_unix_ms) {
    Json result = vehicle_json(snapshot.vehicle);
    result["record_type"] = "snapshot";
    result["recorded_at_unix_ms"] = recorded_at_unix_ms;
    result["elapsed_ms"] = snapshot.elapsed.count();
    result["companion_heartbeat_active"] = snapshot.companion_heartbeat_active;
    result["telemetry_setup"] = {
        {"state", telemetry_state_name(snapshot.telemetry.state)},
        {"completed_requests", snapshot.telemetry.completed_requests},
        {"total_requests", snapshot.telemetry.total_requests},
        {"current_stream", snapshot.telemetry.current_stream},
        {"attempt", snapshot.telemetry.attempt},
        {"failure_result", optional_byte(snapshot.telemetry.failure_result)},
    };
    result["simulated_wind"] =
        snapshot.simulated_wind.has_value()
            ? Json{{"speed_m_s", snapshot.simulated_wind->speed_m_s},
                  {"direction_from_deg",
                      snapshot.simulated_wind->direction_from_deg},
                  {"turbulence_m_s", snapshot.simulated_wind->turbulence_m_s}}
            : Json(nullptr);
    result["companion_link_failsafe"] = {
        {"phase",
            application::companion_link_failsafe_phase_name(
                snapshot.companion_link_failsafe.phase)},
        {"detail", snapshot.companion_link_failsafe.detail},
        {"heartbeat_system_id",
            optional_byte(
                snapshot.companion_link_failsafe.heartbeat_system_id)},
        {"configured_gcs_system_id",
            optional_byte(
                snapshot.companion_link_failsafe.configured_gcs_system_id)},
        {"action",
            snapshot.companion_link_failsafe.action.has_value()
                ? Json(std::string(
                      application::ardupilot_gcs_failsafe_action_name(
                          *snapshot.companion_link_failsafe.action)))
                : Json(nullptr)},
        {"timeout_s",
            optional_number(snapshot.companion_link_failsafe.timeout_s)},
        {"options", optional_number(snapshot.companion_link_failsafe.options)},
        {"parameters_received",
            snapshot.companion_link_failsafe.parameters_received},
        {"parameters_required",
            snapshot.companion_link_failsafe.parameters_required},
    };
    result["camera"] = camera_json(snapshot.camera);
    result["vision"] = vision_json(snapshot.vision);
    result["flight_startup"] = {
        {"phase", startup_phase_name(snapshot.flight_startup.phase)},
        {"detail", snapshot.flight_startup.detail},
        {"target_altitude_m", snapshot.flight_startup.target_altitude_m},
        {"attempt", snapshot.flight_startup.attempt},
    };
    result["autonomy"] = {
        {"phase", autonomy_phase_name(snapshot.autonomy.phase)},
        {"detail", snapshot.autonomy.detail},
        {"vision_landing_target_active",
            snapshot.autonomy.vision_landing_target_active},
        {"terminal_descent_active", snapshot.autonomy.terminal_descent_active},
        {"land_attempt", snapshot.autonomy.land_attempt},
    };
    result["motion_commands_allowed"] = snapshot.motion_commands_allowed;
    result["link_events"] = Json::array();
    for (const auto& event : snapshot.link_events) {
        result["link_events"].push_back(link_event_json(event));
    }
    result["tx_activity"] = link_activity_json(snapshot.tx_activity);
    result["rx_activity"] = link_activity_json(snapshot.rx_activity);
    return result;
}

std::int64_t unix_milliseconds(
    const std::chrono::system_clock::time_point value) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        value.time_since_epoch())
        .count();
}

bool target_tracked(const application::AppSnapshot& snapshot) {
    return snapshot.vision.has_value() &&
           snapshot.vision->target_track.phase ==
               application::TargetTrackPhase::tracking;
}

bool camera_streaming(const application::AppSnapshot& snapshot) {
    return snapshot.camera.has_value() &&
           snapshot.camera->phase ==
               application::ports::CameraSourcePhase::streaming;
}

} // namespace

class JsonDiagnosticSink::Impl {
  public:
    explicit Impl(std::ostream& output) : output_(&output) {}

    explicit Impl(const std::filesystem::path& output_file)
        : owned_output_(output_file, std::ios::app), output_(&owned_output_) {
        if (!owned_output_.is_open()) {
            throw std::runtime_error(
                "cannot open diagnostic log: " + output_file.string());
        }
    }

    void consume(const application::AppSnapshot& snapshot,
        const std::chrono::system_clock::time_point recorded_at) {
        const auto recorded_at_ms = unix_milliseconds(recorded_at);
        write(snapshot_json(snapshot, recorded_at_ms));
        write_events(snapshot, recorded_at_ms);
        previous_ = snapshot;
    }

  private:
    void write(const Json& record) {
        *output_ << record.dump() << '\n' << std::flush;
    }

    void event(const std::string_view name,
        const std::int64_t recorded_at_ms,
        const application::AppSnapshot& snapshot,
        const std::string_view detail,
        Json context = Json::object()) {
        write({
            {"record_type", "event"},
            {"recorded_at_unix_ms", recorded_at_ms},
            {"elapsed_ms", snapshot.elapsed.count()},
            {"event", name},
            {"detail", detail},
            {"context", std::move(context)},
        });
    }

    void write_initial_event(const application::AppSnapshot& snapshot,
        const std::int64_t recorded_at_ms) {
        event("runtime_observation_started",
            recorded_at_ms,
            snapshot,
            "diagnostic sink attached",
            {{"vehicle_connected", snapshot.vehicle.connected},
                {"camera_streaming", camera_streaming(snapshot)},
                {"target_tracked", target_tracked(snapshot)}});
    }

    void write_connection_events(const application::AppSnapshot& previous,
        const application::AppSnapshot& snapshot,
        const std::int64_t recorded_at_ms) {
        if (previous.vehicle.connected != snapshot.vehicle.connected) {
            event(snapshot.vehicle.connected ? "flight_controller_recovered"
                                             : "flight_controller_lost",
                recorded_at_ms,
                snapshot,
                snapshot.vehicle.connected ? "heartbeat recovered"
                                           : "heartbeat timed out");
        }

        const bool was_streaming = camera_streaming(previous);
        const bool is_streaming = camera_streaming(snapshot);
        if (was_streaming != is_streaming) {
            const std::string detail = snapshot.camera.has_value()
                                           ? snapshot.camera->error
                                           : "camera not configured";
            event(is_streaming ? "camera_stream_recovered"
                               : "camera_stream_stalled",
                recorded_at_ms,
                snapshot,
                detail);
        }

        const bool was_tracked = target_tracked(previous);
        const bool is_tracked = target_tracked(snapshot);
        if (was_tracked != is_tracked) {
            event(is_tracked ? "target_acquired" : "target_lost",
                recorded_at_ms,
                snapshot,
                is_tracked ? "confirmed target track"
                           : "confirmed target track unavailable");
        }
    }

    void write_phase_events(const application::AppSnapshot& previous,
        const application::AppSnapshot& snapshot,
        const std::int64_t recorded_at_ms) {
        if (previous.flight_startup.phase != snapshot.flight_startup.phase) {
            event("flight_startup_phase_changed",
                recorded_at_ms,
                snapshot,
                snapshot.flight_startup.detail,
                {{"from", startup_phase_name(previous.flight_startup.phase)},
                    {"to", startup_phase_name(snapshot.flight_startup.phase)}});
        }
        if (previous.autonomy.phase != snapshot.autonomy.phase) {
            event("autonomy_phase_changed",
                recorded_at_ms,
                snapshot,
                snapshot.autonomy.detail,
                {{"from", autonomy_phase_name(previous.autonomy.phase)},
                    {"to", autonomy_phase_name(snapshot.autonomy.phase)}});
        }
        if (previous.companion_link_failsafe.phase !=
            snapshot.companion_link_failsafe.phase) {
            event("companion_link_failsafe_phase_changed",
                recorded_at_ms,
                snapshot,
                snapshot.companion_link_failsafe.detail,
                {{"from",
                     application::companion_link_failsafe_phase_name(
                         previous.companion_link_failsafe.phase)},
                    {"to",
                        application::companion_link_failsafe_phase_name(
                            snapshot.companion_link_failsafe.phase)}});
        }
        if (previous.motion_commands_allowed !=
            snapshot.motion_commands_allowed) {
            event("motion_safety_changed",
                recorded_at_ms,
                snapshot,
                snapshot.motion_commands_allowed ? "motion commands allowed"
                                                 : "motion commands blocked");
        }
    }

    void write_link_events(const application::AppSnapshot& snapshot,
        const std::int64_t recorded_at_ms) {
        for (const auto& link_event : snapshot.link_events) {
            if (link_event.sequence <= last_link_event_sequence_) {
                continue;
            }
            event("mavlink_command_event",
                recorded_at_ms,
                snapshot,
                link_event.detail,
                link_event_json(link_event));
            last_link_event_sequence_ = link_event.sequence;
        }
    }

    void write_events(const application::AppSnapshot& snapshot,
        const std::int64_t recorded_at_ms) {
        if (!previous_.has_value()) {
            write_initial_event(snapshot, recorded_at_ms);
        } else {
            const auto& previous = previous_.value();
            write_connection_events(previous, snapshot, recorded_at_ms);
            write_phase_events(previous, snapshot, recorded_at_ms);
        }
        write_link_events(snapshot, recorded_at_ms);
    }

    std::ofstream owned_output_;
    std::ostream* output_;
    std::optional<application::AppSnapshot> previous_;
    std::uint64_t last_link_event_sequence_{0};
};

JsonDiagnosticSink::JsonDiagnosticSink(std::ostream& output)
    : impl_(std::make_unique<Impl>(output)) {}

JsonDiagnosticSink::JsonDiagnosticSink(const std::filesystem::path& output_file)
    : impl_(std::make_unique<Impl>(output_file)) {}

JsonDiagnosticSink::~JsonDiagnosticSink() = default;

void JsonDiagnosticSink::consume(const application::AppSnapshot& snapshot,
    const std::chrono::system_clock::time_point recorded_at) {
    impl_->consume(snapshot, recorded_at);
}

} // namespace onboard_autonomy::diagnostics::logging
