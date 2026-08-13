#include "onboard_autonomy/application/AppSnapshot.hpp"

#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>

namespace onboard_autonomy::application {
namespace {

std::string_view camera_phase_name(const ports::CameraSourcePhase phase) {
    switch (phase) {
    case ports::CameraSourcePhase::starting:
        return "starting";
    case ports::CameraSourcePhase::streaming:
        return "streaming";
    case ports::CameraSourcePhase::reconnecting:
        return "reconnecting";
    case ports::CameraSourcePhase::stopped:
        return "stopped";
    case ports::CameraSourcePhase::failed:
        return "failed";
    }
    return "failed";
}

std::string_view target_track_phase_name(const TargetTrackPhase phase) {
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

std::string_view telemetry_setup_state_name(const TelemetrySetupState state) {
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

std::string_view flight_startup_phase_name(const FlightStartupPhase phase) {
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

std::string_view autonomy_phase_name(const AutonomyRuntimePhase phase) {
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

std::string json_escape(const std::string_view value) {
    std::ostringstream output;
    for (const char character : value) {
        switch (character) {
        case '\\':
            output << "\\\\";
            break;
        case '"':
            output << "\\\"";
            break;
        case '\n':
            output << "\\n";
            break;
        case '\r':
            output << "\\r";
            break;
        case '\t':
            output << "\\t";
            break;
        default:
            output << character;
            break;
        }
    }
    return output.str();
}

void write_optional_decimal(std::ostringstream& output,
    const std::optional<double>& value) {
    if (value.has_value()) {
        output << std::fixed << std::setprecision(3) << *value;
    } else {
        output << "null";
    }
}

template <typename T>
void write_optional_number(std::ostringstream& output,
    const std::optional<T>& value) {
    if (value.has_value()) {
        output << static_cast<std::uint64_t>(*value);
    } else {
        output << "null";
    }
}

void write_telemetry(std::ostringstream& output, const TelemetryStatus& value) {
    output << "{\"state\":\"" << telemetry_setup_state_name(value.state)
           << "\",\"completed_requests\":" << value.completed_requests
           << ",\"total_requests\":" << value.total_requests
           << ",\"current_stream\":\"" << json_escape(value.current_stream)
           << "\",\"attempt\":" << value.attempt << ",\"failure_result\":";
    write_optional_number(output, value.failure_result);
    output << '}';
}

void write_wind(std::ostringstream& output,
    const std::optional<SimulatedWindProfile>& value) {
    if (!value.has_value()) {
        output << "null";
        return;
    }
    const auto& wind = value.value();
    output << "{\"speed_m_s\":" << std::fixed << std::setprecision(3)
           << wind.speed_m_s
           << ",\"direction_from_deg\":" << wind.direction_from_deg
           << ",\"turbulence_m_s\":" << wind.turbulence_m_s << '}';
}

void write_failsafe(std::ostringstream& output,
    const CompanionLinkFailsafeSnapshot& value) {
    output << "{\"phase\":\"" << companion_link_failsafe_phase_name(value.phase)
           << "\",\"detail\":\"" << json_escape(value.detail)
           << "\",\"heartbeat_system_id\":";
    write_optional_number(output, value.heartbeat_system_id);
    output << ",\"configured_gcs_system_id\":";
    write_optional_number(output, value.configured_gcs_system_id);
    output << ",\"action\":";
    if (value.action.has_value()) {
        output << '"'
               << ardupilot_gcs_failsafe_action_name(value.action.value())
               << '"';
    } else {
        output << "null";
    }
    output << ",\"timeout_s\":";
    write_optional_decimal(output, value.timeout_s);
    output << ",\"options\":";
    write_optional_number(output, value.options);
    output << ",\"parameters_received\":" << value.parameters_received
           << ",\"parameters_required\":" << value.parameters_required << '}';
}

void write_camera(std::ostringstream& output,
    const std::optional<CameraSnapshot>& value) {
    if (!value.has_value()) {
        output << "null";
        return;
    }
    const auto& camera = value.value();
    output << "{\"phase\":\"" << camera_phase_name(camera.phase)
           << "\",\"source\":\"" << json_escape(camera.source)
           << "\",\"error\":\"" << json_escape(camera.error)
           << "\",\"width\":" << camera.width << ",\"height\":" << camera.height
           << ",\"received_frames\":" << camera.received_frames
           << ",\"dropped_before_processing\":"
           << camera.dropped_before_processing
           << ",\"camera_restarts\":" << camera.camera_restarts
           << ",\"frames_with_capture_timestamp\":"
           << camera.frames_with_capture_timestamp << ",\"measured_fps\":";
    write_optional_decimal(output, camera.measured_fps);
    output << ",\"latest_latency_ms\":";
    write_optional_decimal(output, camera.latest_latency_ms);
    output << ",\"average_latency_ms\":";
    write_optional_decimal(output, camera.average_latency_ms);
    output << ",\"maximum_latency_ms\":";
    write_optional_decimal(output, camera.maximum_latency_ms);
    output << ",\"latest_frame_age_ms\":";
    write_optional_decimal(output, camera.latest_frame_age_ms);
    output << '}';
}

void write_target_pose(std::ostringstream& output,
    const std::optional<domain::TargetPose>& value) {
    if (!value.has_value()) {
        output << "null";
        return;
    }
    const auto& pose = value.value();
    output << std::setprecision(4) << "{\"frame\":\"camera_optical\""
           << ",\"right_m\":" << pose.position.right_m
           << ",\"down_m\":" << pose.position.down_m
           << ",\"forward_m\":" << pose.position.forward_m
           << ",\"object_space_error\":" << pose.object_space_error
           << ",\"rotation_tag_to_camera\":[";
    for (std::size_t index = 0; index < pose.rotation_tag_to_camera.size();
         ++index) {
        output << (index == 0U ? "" : ",")
               << pose.rotation_tag_to_camera[index];
    }
    output << "]}";
}

void write_target(std::ostringstream& output,
    const domain::TargetObservation& target) {
    output << "{\"id\":" << target.id << ",\"family\":\""
           << json_escape(target.family) << "\",\"center_x_px\":" << std::fixed
           << std::setprecision(2) << target.center.x_px
           << ",\"center_y_px\":" << target.center.y_px
           << ",\"corrected_bits\":" << target.corrected_bits
           << ",\"decision_margin\":" << target.decision_margin << ",\"pose\":";
    write_target_pose(output, target.pose);
    output << '}';
}

void write_track(std::ostringstream& output, const TargetTrackSnapshot& track) {
    output << "{\"phase\":\"" << target_track_phase_name(track.phase)
           << "\",\"target_id\":";
    if (track.target_id.has_value()) {
        output << track.target_id.value();
    } else {
        output << "null";
    }
    output << ",\"consecutive_observations\":" << track.consecutive_observations
           << ",\"required_observations\":" << track.required_observations
           << ",\"accepted_observations\":" << track.accepted_observations
           << ",\"observation_age_ms\":";
    write_optional_decimal(output, track.observation_age_ms);
    output << ",\"latest_decision_margin\":";
    write_optional_decimal(output, track.latest_decision_margin);
    output << ",\"position\":";
    if (!track.position.has_value()) {
        output << "null}";
        return;
    }
    const auto& position = track.position.value();
    output << "{\"frame\":\"camera_optical\",\"right_m\":";
    write_optional_decimal(output, position.right_m);
    output << ",\"down_m\":";
    write_optional_decimal(output, position.down_m);
    output << ",\"forward_m\":";
    write_optional_decimal(output, position.forward_m);
    output << "}}";
}

void write_vision(std::ostringstream& output,
    const std::optional<VisionSnapshot>& value) {
    if (!value.has_value()) {
        output << "null";
        return;
    }
    const auto& vision = value.value();
    output << "{\"detector\":\"" << json_escape(vision.detector)
           << "\",\"processed_frames\":" << vision.processed_frames
           << ",\"frames_with_targets\":" << vision.frames_with_targets
           << ",\"total_targets\":" << vision.total_targets
           << ",\"latest_processing_ms\":";
    write_optional_decimal(output, vision.latest_processing_ms);
    output << ",\"average_processing_ms\":";
    write_optional_decimal(output, vision.average_processing_ms);
    output << ",\"maximum_processing_ms\":";
    write_optional_decimal(output, vision.maximum_processing_ms);
    output << ",\"last_detection_age_ms\":";
    write_optional_decimal(output, vision.last_detection_age_ms);
    output << ",\"targets\":[";
    for (std::size_t index = 0; index < vision.latest_targets.size(); ++index) {
        if (index > 0U) {
            output << ',';
        }
        write_target(output, vision.latest_targets[index]);
    }
    output << "],\"target_track\":";
    write_track(output, vision.target_track);
    output << '}';
}

} // namespace

std::string AppSnapshot::to_json() const {
    std::string vehicle_json = vehicle.to_json();
    if (!vehicle_json.empty() && vehicle_json.back() == '}') {
        vehicle_json.pop_back();
    }

    std::ostringstream output;
    output << std::boolalpha;
    output << vehicle_json;
    output << ",\"companion_heartbeat_active\":" << companion_heartbeat_active;
    output << ",\"telemetry_setup\":";
    write_telemetry(output, telemetry);
    output << ",\"simulated_wind\":";
    write_wind(output, simulated_wind);
    output << ",\"companion_link_failsafe\":";
    write_failsafe(output, companion_link_failsafe);
    output << ",\"camera\":";
    write_camera(output, camera);
    output << ",\"vision\":";
    write_vision(output, vision);
    output << ",\"flight_startup\":{";
    output << "\"phase\":\"" << flight_startup_phase_name(flight_startup.phase)
           << '\"';
    output << ",\"detail\":\"" << json_escape(flight_startup.detail) << '\"';
    output << ",\"target_altitude_m\":" << flight_startup.target_altitude_m;
    output << ",\"attempt\":" << flight_startup.attempt;
    output << '}';
    output << ",\"autonomy\":{";
    output << "\"phase\":\"" << autonomy_phase_name(autonomy.phase) << '\"';
    output << ",\"detail\":\"" << json_escape(autonomy.detail) << '\"';
    output << ",\"vision_landing_target_active\":"
           << autonomy.vision_landing_target_active;
    output << ",\"terminal_descent_active\":"
           << autonomy.terminal_descent_active;
    output << ",\"land_attempt\":" << autonomy.land_attempt;
    output << '}';
    output << ",\"motion_commands_allowed\":" << motion_commands_allowed;
    output << '}';
    return output.str();
}

} // namespace onboard_autonomy::application
