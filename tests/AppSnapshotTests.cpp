#include "TestCases.hpp"

#include "onboard_autonomy/application/AppSnapshot.hpp"

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void snapshot_keeps_runtime_state_without_camera_data() {
    onboard_autonomy::application::AppSnapshot snapshot;
    snapshot.flight_startup.phase =
        onboard_autonomy::application::
            FlightStartupPhase::waiting_for_vehicle;
    snapshot.flight_startup.detail = "Waiting for controller";
    snapshot.autonomy.phase =
        onboard_autonomy::application::AutonomyRuntimePhase::active;
    snapshot.autonomy.detail = "Autonomy active";
    snapshot.autonomy.vision_landing_target_active = true;
    snapshot.autonomy.terminal_descent_active = true;
    snapshot.motion_commands_allowed = true;
    snapshot.simulated_wind =
        onboard_autonomy::application::SimulatedWindProfile{
            .speed_m_s = 3.0,
            .direction_from_deg = 270.0,
            .turbulence_m_s = 0.6,
        };
    snapshot.companion_heartbeat_active = true;
    snapshot.companion_link_failsafe.phase =
        onboard_autonomy::application::
            CompanionLinkFailsafePhase::accepted;
    snapshot.companion_link_failsafe.action =
        onboard_autonomy::application::
            ArduPilotGcsFailsafeAction::land;
    snapshot.companion_link_failsafe.timeout_s = 3.0;
    snapshot.companion_link_failsafe.options = 0U;
    snapshot.telemetry.state =
        onboard_autonomy::application::TelemetrySetupState::active;
    snapshot.telemetry.completed_requests = 6;
    snapshot.telemetry.total_requests = 6;

    const auto json = nlohmann::json::parse(snapshot.to_json());

    require(json.at("camera").is_null(), "missing camera must be null");
    require(json.at("vision").is_null(), "missing vision must be null");
    require(
        json.at("flight_startup").at("phase") ==
            "waiting_for_vehicle",
        "startup phase must remain in a camera-free snapshot"
    );
    require(
        json.at("autonomy").at("phase") == "active" &&
            json.at("autonomy")
                .at("vision_landing_target_active") == true &&
            json.at("autonomy")
                .at("terminal_descent_active") == true,
        "autonomy state must remain in a camera-free snapshot"
    );
    require(
        json.at("motion_commands_allowed") == true,
        "JSON booleans must not be encoded as integers"
    );
    require(
        json.at("simulated_wind").at("speed_m_s") == 3.0 &&
            json.at("simulated_wind").at("direction_from_deg") ==
                270.0 &&
            json.at("simulated_wind").at("turbulence_m_s") == 0.6,
        "JSON must expose the configured simulation wind profile"
    );
    require(
        json.at("companion_heartbeat_active") == true &&
            json.at("companion_link_failsafe").at("phase") ==
                "accepted" &&
            json.at("companion_link_failsafe").at("action") ==
                "land" &&
            json.at("companion_link_failsafe").at("timeout_s") ==
                3.0,
        "JSON must expose the independently enforced link-loss policy"
    );
    require(
        json.at("telemetry_setup").at("state") == "active" &&
            json.at("telemetry_setup").at("completed_requests") == 6 &&
            json.at("telemetry_setup").at("total_requests") == 6 &&
            json.at("telemetry_setup").at("failure_result").is_null(),
        "JSON must expose acknowledged telemetry setup progress"
    );
}

}  // namespace

void run_app_snapshot_tests() {
    snapshot_keeps_runtime_state_without_camera_data();
}
