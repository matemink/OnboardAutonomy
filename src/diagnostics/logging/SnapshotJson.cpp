#include "SnapshotJson.hpp"

#include <nlohmann/json.hpp>

#include <optional>
#include <utility>

namespace onboard_autonomy::diagnostics::logging::detail {

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
    const mission::ports::CameraSourcePhase phase) {
    using mission::ports::CameraSourcePhase;
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

std::string_view telemetry_state_name(
    const mission::TelemetrySetupState state) {
    using mission::TelemetrySetupState;
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

std::string_view link_direction_name(
    const mission::LinkEventDirection direction) {
    return direction == mission::LinkEventDirection::outbound ? "outbound"
                                                              : "inbound";
}

std::string_view link_status_name(const mission::LinkEventStatus status) {
    using mission::LinkEventStatus;
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

Json vehicle_json(const mission::VehicleSnapshot& vehicle) {
    Json result{
        {"connected", vehicle.connected},
        {"gps_ready", vehicle.gps_ready},
        {"navigation_ready", vehicle.navigation_ready},
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
        {"yaw_rate_rad_per_second",
            optional_number(vehicle.yaw_rate_rad_per_second)},
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

Json camera_json(const std::optional<mission::CameraSnapshot>& camera) {
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

Json target_json(const mission::TargetObservation& target) {
    return Json{
        {"id", target.id},
        {"family", target.family},
        {"center_x_px", target.center.x_px},
        {"center_y_px", target.center.y_px},
        {"confidence_percent", target.confidence_percent},
    };
}

Json vision_json(const std::optional<mission::VisionSnapshot>& vision) {
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
    };
}

Json link_event_json(const mission::LinkEvent& event) {
    return Json{
        {"sequence", event.sequence},
        {"elapsed_ms", event.elapsed.count()},
        {"direction", link_direction_name(event.direction)},
        {"status", link_status_name(event.status)},
        {"label", event.label},
        {"detail", event.detail},
    };
}

Json link_activity_json(const std::optional<mission::LinkActivity>& activity) {
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

Json snapshot_json(const mission::AppSnapshot& snapshot,
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
            mission::companion_link_failsafe_phase_name(
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
                ? Json(std::string(mission::ardupilot_gcs_failsafe_action_name(
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
    result["link_events"] = Json::array();
    for (const auto& event : snapshot.link_events) {
        result["link_events"].push_back(link_event_json(event));
    }
    result["tx_activity"] = link_activity_json(snapshot.tx_activity);
    result["rx_activity"] = link_activity_json(snapshot.rx_activity);
    return result;
}

} // namespace onboard_autonomy::diagnostics::logging::detail
