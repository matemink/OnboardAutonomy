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
    require(json.at("navigation_ready") == true &&
                json.at("gps_ready") == false &&
                json.at("simulated_wind").at("speed_m_s") == 3.0 &&
                json.at("companion_link_failsafe").at("action") == "land" &&
                json.at("telemetry_setup").at("completed_requests") == 6,
        "snapshot diagnostics must preserve safety and setup state");
    require(!json.contains("flight_startup") && !json.contains("autonomy") &&
                !json.contains("motion_commands_allowed") &&
                !json.contains("aerial_tracking_available"),
        "observation diagnostics must not retain the removed control schema");
}

void snapshot_serializes_camera_and_object_detections() {
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
        .detector = "synthetic detector",
        .processed_frames = 10,
        .frames_with_targets = 1,
        .total_targets = 1,
        .latest_processing_ms = 5.5,
        .average_processing_ms = 5.0,
        .maximum_processing_ms = 6.0,
        .last_detection_age_ms = 20.0,
        .latest_targets = {{
            .id = 0,
            .family = "object",
            .center = {.x_px = 160.0, .y_px = 120.0},
            .corners = {},

            .confidence_percent = 80.0,
        }},
    };

    const auto json = serialize_snapshot(snapshot);
    require(json.at("connected") == false &&
                json.at("camera").at("phase") == "streaming" &&
                json.at("camera").at("source") == "Camera Module 3 \"Wide\"" &&
                json.at("camera").at("latest_latency_ms") == 21.5,
        "diagnostic adapter must preserve camera state and JSON escaping");
    const auto& vision = json.at("vision");
    const auto& target = vision.at("targets").at(0);
    require(vision.at("processed_frames") == 10 && target.at("id") == 0 &&
                target.at("confidence_percent") == 80.0,
        "diagnostics must preserve object detections");
    require(!vision.contains("target_track") && !target.contains("pose") &&
                !target.contains("corrected_bits"),
        "diagnostics must not retain the removed marker-landing schema");
}

void transition_events_reconstruct_runtime_failures() {
    using namespace std::chrono_literals;
    std::ostringstream output;
    JsonDiagnosticSink sink{output};

    AppSnapshot waiting;
    waiting.elapsed = 10ms;
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
    active.companion_link_failsafe.phase =
        onboard_autonomy::mission::CompanionLinkFailsafePhase::accepted;
    active.companion_link_failsafe.detail = "failsafe parameters accepted";
    active.link_events.push_back({
        .sequence = 1,
        .elapsed = 100ms,
        .direction = onboard_autonomy::mission::LinkEventDirection::outbound,
        .status = onboard_autonomy::mission::LinkEventStatus::success,
        .label = "TELEMETRY",
        .detail = "COMMAND_ACK ACCEPTED",
    });
    sink.consume(active, std::chrono::system_clock::time_point{2s});
    sink.consume_link_event(active.link_events.back(),
        std::chrono::system_clock::time_point{2s});
    sink.consume_failsafe_transition(waiting.companion_link_failsafe.phase,
        active.companion_link_failsafe, active.elapsed,
        std::chrono::system_clock::time_point{2s});

    AppSnapshot failed = active;
    failed.elapsed = 300ms;
    failed.vehicle.connected = false;
    failed.camera->phase =
        onboard_autonomy::mission::ports::CameraSourcePhase::reconnecting;
    failed.camera->error = "frame stalled";
    failed.companion_link_failsafe.phase =
        onboard_autonomy::mission::CompanionLinkFailsafePhase::rejected;
    failed.companion_link_failsafe.detail = "failsafe activation failed";
    failed.link_events.push_back({
        .sequence = 2,
        .elapsed = 280ms,
        .direction = onboard_autonomy::mission::LinkEventDirection::inbound,
        .status = onboard_autonomy::mission::LinkEventStatus::failure,
        .label = "TELEMETRY",
        .detail = "COMMAND_ACK DENIED",
    });
    sink.consume(failed, std::chrono::system_clock::time_point{3s});
    sink.consume_link_event(failed.link_events.back(),
        std::chrono::system_clock::time_point{3s});
    sink.consume_failsafe_transition(active.companion_link_failsafe.phase,
        failed.companion_link_failsafe, failed.elapsed,
        std::chrono::system_clock::time_point{3s});

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
                has_event("camera_stream_stalled"),
        "diagnostics must record hardware transitions");
    require(has_event("companion_link_failsafe_phase_changed") &&
                has_event("mavlink_command_event"),
        "diagnostics must record failsafe and protocol transitions");
}

} // namespace

void run_json_diagnostic_sink_tests() {
    snapshot_keeps_runtime_state_without_camera_data();
    snapshot_serializes_camera_and_object_detections();
    transition_events_reconstruct_runtime_failures();
}
