#include "TestCases.hpp"

#include "onboard_autonomy/mission/AppSnapshot.hpp"
#include "onboard_autonomy/diagnostics/logging/JsonDiagnosticSink.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Json = nlohmann::json;
using onboard_autonomy::diagnostics::logging::JsonDiagnosticSink;
using onboard_autonomy::mission::AppSnapshot;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::vector<Json> parse_records(const std::string& output) {
    std::vector<Json> records;
    std::istringstream lines{output};
    for (std::string line; std::getline(lines, line);) {
        if (!line.empty()) {
            records.push_back(Json::parse(line));
        }
    }
    return records;
}

Json serialize_snapshot(const AppSnapshot& snapshot) {
    std::ostringstream output;
    JsonDiagnosticSink sink{output};
    sink.consume(snapshot,
        std::chrono::system_clock::time_point{std::chrono::seconds{123}});
    return parse_records(output.str()).front();
}

void snapshot_keeps_runtime_state_without_camera_data() {
    AppSnapshot snapshot;
    snapshot.flight_startup.phase =
        onboard_autonomy::mission::FlightStartupPhase::waiting_for_vehicle;
    snapshot.flight_startup.detail = "Waiting for controller";
    snapshot.autonomy.phase =
        onboard_autonomy::mission::AutonomyRuntimePhase::active;
    snapshot.autonomy.detail = "Autonomy active";
    snapshot.autonomy.vision_landing_target_active = true;
    snapshot.autonomy.terminal_descent_active = true;
    snapshot.motion_commands_allowed = true;
    snapshot.precision_landing_available = true;
    snapshot.aerial_tracking_available = true;
    snapshot.vehicle.gps_ready = false;
    snapshot.vehicle.navigation_ready = true;
    snapshot.simulated_wind = onboard_autonomy::mission::SimulatedWindProfile{
        .speed_m_s = 3.0,
        .direction_from_deg = 270.0,
        .turbulence_m_s = 0.6,
    };
    snapshot.companion_heartbeat_active = true;
    snapshot.companion_link_failsafe.phase =
        onboard_autonomy::mission::CompanionLinkFailsafePhase::accepted;
    snapshot.companion_link_failsafe.action =
        onboard_autonomy::mission::ArduPilotGcsFailsafeAction::land;
    snapshot.companion_link_failsafe.timeout_s = 3.0;
    snapshot.companion_link_failsafe.options = 0U;
    snapshot.telemetry.state =
        onboard_autonomy::mission::TelemetrySetupState::active;
    snapshot.telemetry.completed_requests = 6;
    snapshot.telemetry.total_requests = 6;

    const auto json = serialize_snapshot(snapshot);

    require(json.at("record_type") == "snapshot" &&
                json.at("recorded_at_unix_ms") == 123000,
        "diagnostic snapshots must carry record type and wall-clock time");
    require(json.at("camera").is_null() && json.at("vision").is_null(),
        "missing camera and vision data must remain explicit nulls");
    require(json.at("flight_startup").at("phase") == "waiting_for_vehicle" &&
                json.at("autonomy").at("phase") == "active" &&
                json.at("autonomy").at("vision_landing_target_active") &&
                json.at("autonomy").at("terminal_descent_active"),
        "snapshot diagnostics must preserve mission runtime state");
    require(json.at("motion_commands_allowed") == true &&
                json.at("precision_landing_available") == true &&
                json.at("aerial_tracking_available") == true &&
                json.at("navigation_ready") == true &&
                json.at("gps_ready") == false &&
                json.at("simulated_wind").at("speed_m_s") == 3.0 &&
                json.at("companion_link_failsafe").at("action") == "land" &&
                json.at("telemetry_setup").at("completed_requests") == 6,
        "snapshot diagnostics must preserve safety and setup state");
}

void snapshot_serializes_camera_and_metric_vision() {
    AppSnapshot snapshot;
    snapshot.camera = onboard_autonomy::mission::CameraSnapshot{
        .phase = onboard_autonomy::mission::ports::CameraSourcePhase::streaming,
        .source = "Camera Module 3 \"Wide\"",
        .error = "",
        .width = 640,
        .height = 480,
        .received_frames = 5,
        .dropped_before_processing = 0,
        .camera_restarts = 2,
        .frames_with_capture_timestamp = 5,
        .measured_fps = std::nullopt,
        .latest_latency_ms = 21.5,
        .average_latency_ms = std::nullopt,
        .maximum_latency_ms = std::nullopt,
        .latest_frame_age_ms = std::nullopt,
    };
    snapshot.vision = onboard_autonomy::mission::VisionSnapshot{
        .detector = "AprilTag 3 / tagStandard41h12",
        .processed_frames = 10,
        .frames_with_targets = 1,
        .total_targets = 1,
        .latest_processing_ms = 5.5,
        .average_processing_ms = 5.0,
        .maximum_processing_ms = 6.0,
        .last_detection_age_ms = 20.0,
        .latest_targets = {{
            .id = 0,
            .family = "tagStandard41h12",
            .center = {.x_px = 160.0, .y_px = 120.0},
            .corners = {},
            .corrected_bits = 0,
            .decision_margin = 80.0,
            .pose =
                onboard_autonomy::mission::TargetPose{
                    .position = {.right_m = 0.1,
                        .down_m = -0.2,
                        .forward_m = 1.25},
                    .rotation_tag_to_camera =
                        {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0},
                    .object_space_error = 0.003,
                },
        }},
        .target_track =
            {
                .phase = onboard_autonomy::mission::TargetTrackPhase::tracking,
                .target_id = 0,
                .consecutive_observations = 4,
                .required_observations = 3,
                .accepted_observations = 4,
                .observation_age_ms = 12.0,
                .latest_decision_margin = 80.0,
                .position =
                    onboard_autonomy::mission::CameraFramePosition{
                        .right_m = 0.09,
                        .down_m = -0.18,
                        .forward_m = 1.20,
                    },
            },
    };

    const auto json = serialize_snapshot(snapshot);
    require(json.at("connected") == false &&
                json.at("camera").at("phase") == "streaming" &&
                json.at("camera").at("source") == "Camera Module 3 \"Wide\"" &&
                json.at("camera").at("latest_latency_ms") == 21.5,
        "diagnostic adapter must preserve camera state and JSON escaping");
    require(json.at("vision").at("processed_frames") == 10 &&
                json.at("vision").at("targets").at(0).at("id") == 0 &&
                json.at("vision").at("targets").at(0).at("pose").at(
                    "forward_m") == 1.25 &&
                json.at("vision").at("target_track").at("phase") == "tracking",
        "diagnostic adapter must preserve typed metric vision results");
}

void transition_events_reconstruct_runtime_failures() {
    using namespace std::chrono_literals;
    std::ostringstream output;
    JsonDiagnosticSink sink{output};

    AppSnapshot waiting;
    waiting.elapsed = 10ms;
    waiting.flight_startup.phase =
        onboard_autonomy::mission::FlightStartupPhase::waiting_for_vehicle;
    waiting.autonomy.phase =
        onboard_autonomy::mission::AutonomyRuntimePhase::waiting_for_startup;
    waiting.camera = onboard_autonomy::mission::CameraSnapshot{
        .phase =
            onboard_autonomy::mission::ports::CameraSourcePhase::reconnecting,
        .source = "test camera",
        .error = "stream unavailable",
        .width = 0,
        .height = 0,
        .received_frames = 0,
        .dropped_before_processing = 0,
        .camera_restarts = 0,
        .frames_with_capture_timestamp = 0,
        .measured_fps = std::nullopt,
        .latest_latency_ms = std::nullopt,
        .average_latency_ms = std::nullopt,
        .maximum_latency_ms = std::nullopt,
        .latest_frame_age_ms = std::nullopt,
    };
    waiting.vision = onboard_autonomy::mission::VisionSnapshot{};
    sink.consume(waiting, std::chrono::system_clock::time_point{1s});

    AppSnapshot active = waiting;
    active.elapsed = 120ms;
    active.vehicle.connected = true;
    active.camera->phase =
        onboard_autonomy::mission::ports::CameraSourcePhase::streaming;
    active.camera->error.clear();
    active.vision->target_track.phase =
        onboard_autonomy::mission::TargetTrackPhase::tracking;
    active.flight_startup.phase =
        onboard_autonomy::mission::FlightStartupPhase::setting_guided;
    active.flight_startup.detail = "GUIDED command accepted";
    active.autonomy.phase =
        onboard_autonomy::mission::AutonomyRuntimePhase::active;
    active.autonomy.detail = "precision landing active";
    active.companion_link_failsafe.phase =
        onboard_autonomy::mission::CompanionLinkFailsafePhase::accepted;
    active.companion_link_failsafe.detail = "failsafe parameters accepted";
    active.motion_commands_allowed = true;
    active.link_events.push_back({
        .sequence = 1,
        .elapsed = 100ms,
        .direction = onboard_autonomy::mission::LinkEventDirection::outbound,
        .status = onboard_autonomy::mission::LinkEventStatus::success,
        .label = "GUIDED",
        .detail = "COMMAND_ACK ACCEPTED",
    });
    sink.consume(active, std::chrono::system_clock::time_point{2s});

    AppSnapshot failed = active;
    failed.elapsed = 300ms;
    failed.vehicle.connected = false;
    failed.camera->phase =
        onboard_autonomy::mission::ports::CameraSourcePhase::reconnecting;
    failed.camera->error = "frame stalled";
    failed.vision->target_track.phase =
        onboard_autonomy::mission::TargetTrackPhase::searching;
    failed.flight_startup.phase =
        onboard_autonomy::mission::FlightStartupPhase::failed;
    failed.flight_startup.detail = "arm command rejected";
    failed.autonomy.phase =
        onboard_autonomy::mission::AutonomyRuntimePhase::failed;
    failed.autonomy.detail = "controller heartbeat was lost";
    failed.companion_link_failsafe.phase =
        onboard_autonomy::mission::CompanionLinkFailsafePhase::rejected;
    failed.companion_link_failsafe.detail = "failsafe activation failed";
    failed.motion_commands_allowed = false;
    failed.link_events.push_back({
        .sequence = 2,
        .elapsed = 280ms,
        .direction = onboard_autonomy::mission::LinkEventDirection::inbound,
        .status = onboard_autonomy::mission::LinkEventStatus::failure,
        .label = "ARM",
        .detail = "COMMAND_ACK DENIED",
    });
    sink.consume(failed, std::chrono::system_clock::time_point{3s});

    const auto records = parse_records(output.str());
    std::vector<std::string> events;
    for (const auto& record : records) {
        if (record.at("record_type") == "event") {
            events.push_back(record.at("event").get<std::string>());
        }
    }
    const auto has_event = [&events](const std::string& expected) {
        return std::find(events.begin(), events.end(), expected) !=
               events.end();
    };

    require(has_event("flight_controller_recovered") &&
                has_event("flight_controller_lost") &&
                has_event("camera_stream_recovered") &&
                has_event("camera_stream_stalled") &&
                has_event("target_acquired") && has_event("target_lost"),
        "diagnostics must record hardware and target transitions");
    require(has_event("flight_startup_phase_changed") &&
                has_event("autonomy_phase_changed") &&
                has_event("companion_link_failsafe_phase_changed") &&
                has_event("motion_safety_changed") &&
                has_event("mavlink_command_event"),
        "diagnostics must record mission, safety, and command transitions");
}

} // namespace

void run_json_diagnostic_sink_tests() {
    snapshot_keeps_runtime_state_without_camera_data();
    snapshot_serializes_camera_and_metric_vision();
    transition_events_reconstruct_runtime_failures();
}
