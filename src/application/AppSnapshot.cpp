#include "onboard_autonomy/application/AppSnapshot.hpp"

#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>

namespace onboard_autonomy::application {
namespace {

std::string_view camera_phase_name(
    const ports::CameraSourcePhase phase
) {
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

std::string_view target_track_phase_name(
    const TargetTrackPhase phase
) {
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

std::string_view flight_startup_phase_name(
    const FlightStartupPhase phase
) {
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
    const AutonomyRuntimePhase phase
) {
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

void write_optional_decimal(
    std::ostringstream& output,
    const std::optional<double>& value
) {
    if (value.has_value()) {
        output << std::fixed << std::setprecision(3) << *value;
    } else {
        output << "null";
    }
}

template <typename T>
void write_optional_number(
    std::ostringstream& output,
    const std::optional<T>& value
) {
    if (value.has_value()) {
        output << static_cast<std::uint64_t>(*value);
    } else {
        output << "null";
    }
}

}  // namespace

std::string AppSnapshot::to_json() const {
    std::string vehicle_json = vehicle.to_json();
    if (!vehicle_json.empty() && vehicle_json.back() == '}') {
        vehicle_json.pop_back();
    }

    std::ostringstream output;
    output << std::boolalpha;
    output << vehicle_json;
    output << ",\"companion_heartbeat_active\":"
           << companion_heartbeat_active;
    output << ",\"companion_link_failsafe\":{";
    output << "\"phase\":\""
           << companion_link_failsafe_phase_name(
                  companion_link_failsafe.phase
              )
           << '"';
    output << ",\"detail\":\""
           << json_escape(companion_link_failsafe.detail) << '"';
    output << ",\"heartbeat_system_id\":";
    write_optional_number(
        output,
        companion_link_failsafe.heartbeat_system_id
    );
    output << ",\"configured_gcs_system_id\":";
    write_optional_number(
        output,
        companion_link_failsafe.configured_gcs_system_id
    );
    output << ",\"action\":";
    if (companion_link_failsafe.action.has_value()) {
        output << '"'
               << ardupilot_gcs_failsafe_action_name(
                      *companion_link_failsafe.action
                  )
               << '"';
    } else {
        output << "null";
    }
    output << ",\"timeout_s\":";
    write_optional_decimal(
        output,
        companion_link_failsafe.timeout_s
    );
    output << ",\"options\":";
    write_optional_number(
        output,
        companion_link_failsafe.options
    );
    output << ",\"parameters_received\":"
           << companion_link_failsafe.parameters_received;
    output << ",\"parameters_required\":"
           << companion_link_failsafe.parameters_required;
    output << '}';
    output << ",\"camera\":";
    if (!camera.has_value()) {
        output << "null";
    } else {
        output << '{';
        output << "\"phase\":\""
               << camera_phase_name(camera->phase) << '"';
        output << ",\"source\":\""
               << json_escape(camera->source) << '"';
        output << ",\"error\":\""
               << json_escape(camera->error) << '"';
        output << ",\"width\":" << camera->width;
        output << ",\"height\":" << camera->height;
        output << ",\"received_frames\":"
               << camera->received_frames;
        output << ",\"dropped_before_processing\":"
               << camera->dropped_before_processing;
        output << ",\"camera_restarts\":"
               << camera->camera_restarts;
        output << ",\"frames_with_capture_timestamp\":"
               << camera->frames_with_capture_timestamp;
        output << ",\"measured_fps\":";
        write_optional_decimal(output, camera->measured_fps);
        output << ",\"latest_latency_ms\":";
        write_optional_decimal(output, camera->latest_latency_ms);
        output << ",\"average_latency_ms\":";
        write_optional_decimal(output, camera->average_latency_ms);
        output << ",\"maximum_latency_ms\":";
        write_optional_decimal(output, camera->maximum_latency_ms);
        output << ",\"latest_frame_age_ms\":";
        write_optional_decimal(output, camera->latest_frame_age_ms);
        output << '}';
    }
    output << ",\"vision\":";
    if (!vision.has_value()) {
        output << "null";
    } else {
        output << '{';
        output << "\"detector\":\""
               << json_escape(vision->detector) << '"';
        output << ",\"processed_frames\":"
               << vision->processed_frames;
        output << ",\"frames_with_targets\":"
               << vision->frames_with_targets;
        output << ",\"total_targets\":"
               << vision->total_targets;
        output << ",\"latest_processing_ms\":";
        write_optional_decimal(output, vision->latest_processing_ms);
        output << ",\"average_processing_ms\":";
        write_optional_decimal(output, vision->average_processing_ms);
        output << ",\"maximum_processing_ms\":";
        write_optional_decimal(output, vision->maximum_processing_ms);
        output << ",\"last_detection_age_ms\":";
        write_optional_decimal(output, vision->last_detection_age_ms);
        output << ",\"targets\":[";
        for (std::size_t index = 0;
             index < vision->latest_targets.size();
             ++index) {
            if (index > 0U) {
                output << ',';
            }
            const auto& target = vision->latest_targets[index];
            output << '{';
            output << "\"id\":" << target.id;
            output << ",\"family\":\""
                   << json_escape(target.family) << '"';
            output << ",\"center_x_px\":" << std::fixed
                   << std::setprecision(2) << target.center.x_px;
            output << ",\"center_y_px\":" << target.center.y_px;
            output << ",\"corrected_bits\":"
                   << target.corrected_bits;
            output << ",\"decision_margin\":"
                   << target.decision_margin;
            output << ",\"pose\":";
            if (!target.pose.has_value()) {
                output << "null";
            } else {
                output << std::setprecision(4);
                output << "{\"frame\":\"camera_optical\"";
                output << ",\"right_m\":"
                       << target.pose->position.right_m;
                output << ",\"down_m\":"
                       << target.pose->position.down_m;
                output << ",\"forward_m\":"
                       << target.pose->position.forward_m;
                output << ",\"object_space_error\":"
                       << target.pose->object_space_error;
                output << ",\"rotation_tag_to_camera\":[";
                for (std::size_t rotation_index = 0;
                     rotation_index <
                         target.pose->rotation_tag_to_camera.size();
                     ++rotation_index) {
                    if (rotation_index > 0U) {
                        output << ',';
                    }
                    output << target.pose
                                  ->rotation_tag_to_camera[rotation_index];
                }
                output << "]}";
            }
            output << '}';
        }
        output << "],\"target_track\":{";
        output << "\"phase\":\""
               << target_track_phase_name(vision->target_track.phase)
               << '"';
        output << ",\"target_id\":";
        if (vision->target_track.target_id.has_value()) {
            output << *vision->target_track.target_id;
        } else {
            output << "null";
        }
        output << ",\"consecutive_observations\":"
               << vision->target_track.consecutive_observations;
        output << ",\"required_observations\":"
               << vision->target_track.required_observations;
        output << ",\"accepted_observations\":"
               << vision->target_track.accepted_observations;
        output << ",\"observation_age_ms\":";
        write_optional_decimal(
            output,
            vision->target_track.observation_age_ms
        );
        output << ",\"latest_decision_margin\":";
        write_optional_decimal(
            output,
            vision->target_track.latest_decision_margin
        );
        output << ",\"position\":";
        if (!vision->target_track.position.has_value()) {
            output << "null";
        } else {
            output << "{\"frame\":\"camera_optical\"";
            output << ",\"right_m\":";
            write_optional_decimal(
                output,
                vision->target_track.position->right_m
            );
            output << ",\"down_m\":";
            write_optional_decimal(
                output,
                vision->target_track.position->down_m
            );
            output << ",\"forward_m\":";
            write_optional_decimal(
                output,
                vision->target_track.position->forward_m
            );
            output << '}';
        }
        output << "}}";
    }
    output << ",\"flight_startup\":{";
    output << "\"phase\":\""
           << flight_startup_phase_name(flight_startup.phase)
           << '\"';
    output << ",\"detail\":\""
           << json_escape(flight_startup.detail) << '\"';
    output << ",\"target_altitude_m\":"
           << flight_startup.target_altitude_m;
    output << ",\"attempt\":" << flight_startup.attempt;
    output << '}';
    output << ",\"autonomy\":{";
    output << "\"phase\":\""
           << autonomy_phase_name(autonomy.phase) << '\"';
    output << ",\"detail\":\""
           << json_escape(autonomy.detail) << '\"';
    output << ",\"vision_landing_target_active\":"
           << autonomy.vision_landing_target_active;
    output << ",\"terminal_descent_active\":"
           << autonomy.terminal_descent_active;
    output << ",\"land_attempt\":" << autonomy.land_attempt;
    output << '}';
    output << ",\"motion_commands_allowed\":"
           << motion_commands_allowed;
    output << '}';
    return output.str();
}

}  // namespace onboard_autonomy::application
