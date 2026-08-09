#include "TestCases.hpp"

#include "onboard_autonomy/adapters/ardupilot/BoardTypeCatalog.hpp"
#include "onboard_autonomy/presentation/console/ConsoleView.hpp"

#include <chrono>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void healthy_snapshot_is_human_readable() {
    onboard_autonomy::application::AppSnapshot app_snapshot;
    auto& snapshot = app_snapshot.vehicle;
    snapshot.connected = true;
    snapshot.gps_ready = true;
    snapshot.battery_ready = true;
    snapshot.system_health_known = true;
    snapshot.system_health_ok = true;
    snapshot.armable = true;
    snapshot.armed = false;
    snapshot.system_id = 1;
    snapshot.component_id = 1;
    snapshot.vehicle_type = 2;
    snapshot.autopilot_type = 3;
    snapshot.system_status = 3;
    snapshot.gps_fix_type = 6;
    snapshot.satellites_visible = 10;
    snapshot.battery_voltage_v = 12.6;
    snapshot.battery_current_a = 0.0;
    snapshot.battery_remaining_pct = 100;
    snapshot.autopilot_metadata =
        onboard_autonomy::domain::AutopilotMetadata{
            .firmware_major = 4,
            .firmware_minor = 6,
            .firmware_patch = 3,
            .firmware_release_type = 255,
            .capabilities = 0,
            .board_version = (56U << 16U) | 2U,
            .vendor_id = 0x1209,
            .product_id = 0x5740,
        };
    app_snapshot.companion_heartbeat_active = true;
    app_snapshot.companion_link_failsafe.phase =
        onboard_autonomy::application::
            CompanionLinkFailsafePhase::accepted;
    app_snapshot.companion_link_failsafe.action =
        onboard_autonomy::application::
            ArduPilotGcsFailsafeAction::land;
    app_snapshot.companion_link_failsafe.timeout_s = 3.0;
    app_snapshot.companion_link_failsafe.configured_gcs_system_id = 1;
    app_snapshot.telemetry.state =
        onboard_autonomy::application::TelemetrySetupState::active;
    app_snapshot.telemetry.completed_requests = 6;
    app_snapshot.telemetry.total_requests = 6;
    app_snapshot.simulated_wind =
        onboard_autonomy::application::SimulatedWindProfile{
            .speed_m_s = 3.0,
            .direction_from_deg = 270.0,
            .turbulence_m_s = 0.6,
        };

    std::istringstream board_table{
        "Reserved \"PX4 [BL] FMU v6C.x\" 56\n"
    };
    const auto board_catalog =
        onboard_autonomy::adapters::ardupilot::
            BoardTypeCatalog::from_stream(board_table);
    const auto output = onboard_autonomy::presentation::console::render_console(
        app_snapshot,
        "udp://127.0.0.1:14550",
        false,
        &board_catalog
    );

    require(
        output.find("[ READY ]") != std::string::npos,
        "overall readiness must be prominent"
    );
    require(
        output.find("GPS RTK FIXED / 10 SAT") != std::string::npos,
        "GPS details must be readable"
    );
    require(
        output.find("BAT 12.60 V / 100%") != std::string::npos,
        "battery details must be readable"
    );
    require(
        output.find("SIM WIND") == std::string::npos,
        "simulation weather belongs to the Gazebo HUD"
    );
    require(
        output.find("TELEMETRY READY / 6 STREAMS") !=
                std::string::npos &&
            output.find("FIRMWARE 4.6.3 OFFICIAL") !=
                std::string::npos &&
            output.find(
                "PX4 [BL] FMU v6C.x / ID 56 / SILICON 2"
            ) !=
                std::string::npos,
        "acknowledged telemetry and reported controller metadata "
        "must be visible"
    );
    require(
        output.find("RASPBERRY PI 5") != std::string::npos &&
            output.find("PX4 [BL] FMU v6C.x") !=
                std::string::npos,
        "the target hardware names must be visible"
    );
    require(
        output.find("NO ACTIVE WARNINGS") != std::string::npos,
        "healthy state must remain visible without a log section"
    );
    require(
        output.find(
            "LINK FAILSAFE READY / ARDUPILOT LAND / 3.0 S / SYSID 1"
        ) != std::string::npos,
        "operator view must expose the flight-controller-owned fallback"
    );
}

void disconnected_snapshot_is_waiting() {
    const onboard_autonomy::application::AppSnapshot snapshot;
    const auto output = onboard_autonomy::presentation::console::render_console(
        snapshot,
        "udp://127.0.0.1:14550",
        false
    );

    require(
        output.find("WAITING FOR FLIGHT CONTROLLER") != std::string::npos,
        "disconnected state must not look like a failure"
    );
}

void command_bus_shows_both_directions() {
    onboard_autonomy::application::AppSnapshot snapshot;
    snapshot.motion_commands_allowed = true;
    snapshot.flight_startup.phase =
        onboard_autonomy::application::FlightStartupPhase::completed;
    snapshot.flight_startup.detail = "Takeoff complete";
    snapshot.autonomy.phase =
        onboard_autonomy::application::AutonomyRuntimePhase::active;
    snapshot.autonomy.detail = "Vision target F/R/D 0.4/-0.2/8.1 m";
    snapshot.elapsed = std::chrono::milliseconds(1200);
    snapshot.tx_activity = onboard_autonomy::application::LinkActivity{
        .sequence = 1,
        .observed_at = std::chrono::milliseconds(1100),
        .message_name = "COMMAND_LONG",
        .detail = "SET_MODE",
    };
    snapshot.rx_activity = onboard_autonomy::application::LinkActivity{
        .sequence = 2,
        .observed_at = std::chrono::milliseconds(1100),
        .message_name = "COMMAND_ACK",
        .detail = "SET_MODE ACCEPTED",
    };

    const auto output = onboard_autonomy::presentation::console::render_console(
        snapshot,
        "udp://127.0.0.1:14550",
        false
    );

    require(
        output.find("RASPBERRY PI 5") != std::string::npos &&
            output.find("FLIGHT CONTROLLER") != std::string::npos,
        "command bus endpoints must be visible"
    );
    require(
        output.find("==[ COMMAND_LONG: SET_MODE ]") !=
                std::string::npos &&
            output.find("[ COMMAND_ACK: SET_MODE ACCEPTED ]==") !=
                std::string::npos,
        "command bus must show the actual MAVLink frame in each direction"
    );
    require(
        output.find("SET_MODE") != std::string::npos &&
            output.find("ACCEPTED") != std::string::npos,
        "command and acknowledgement labels must be visible"
    );
    require(
        output.find("[S] START AGAIN") != std::string::npos &&
            output.find("[Q] QUIT") != std::string::npos,
        "interactive command hints must be visible"
    );
    require(
        output.find("AUTONOMY: ACTIVE") != std::string::npos &&
            output.find("STARTUP: COMPLETE") != std::string::npos &&
            output.find("Vision target F/R/D") != std::string::npos,
        "production startup and runtime state must be visible"
    );

    snapshot.elapsed = std::chrono::milliseconds(1320);
    const auto dim_pulse =
        onboard_autonomy::presentation::console::render_console(
            snapshot,
            "udp://127.0.0.1:14550",
            false
        );
    require(
        dim_pulse.find("--[ COMMAND_LONG: SET_MODE ]") !=
            std::string::npos,
        "a fresh frame must alternate to the dim blink phase"
    );

    snapshot.elapsed = std::chrono::milliseconds(1800);
    const auto stale =
        onboard_autonomy::presentation::console::render_console(
            snapshot,
            "udp://127.0.0.1:14550",
            false
        );
    require(
        stale.find("COMMAND_LONG") == std::string::npos &&
            stale.find("COMMAND_ACK") == std::string::npos,
        "expired traffic must disappear instead of looking current"
    );
}

void colors_group_related_elements() {
    const onboard_autonomy::application::AppSnapshot snapshot;
    const auto output = onboard_autonomy::presentation::console::render_console(
        snapshot,
        "udp://127.0.0.1:14550",
        true
    );

    require(
        output.find(
            "\x1b[96m|   RASPBERRY PI 5   |\x1b[0m"
        ) != std::string::npos,
        "Raspberry Pi must use the companion color"
    );
    require(
        output.find(
            "\x1b[93m| FLIGHT CONTROLLER  |\x1b[0m"
        ) != std::string::npos,
        "Pixhawk must use the controller color"
    );
    require(
        output.find("\x1b[94m+===") != std::string::npos,
        "screen chrome must have its own color"
    );
}

void long_mavlink_message_names_are_safely_clipped() {
    onboard_autonomy::application::AppSnapshot snapshot;
    snapshot.elapsed = std::chrono::milliseconds(1200);
    snapshot.tx_activity = onboard_autonomy::application::LinkActivity{
        .sequence = 1,
        .observed_at = std::chrono::milliseconds(1200),
        .message_name =
            "OPEN_DRONE_ID_MESSAGE_PACK_REGISTRATION",
        .detail = "TRANSMITTED",
    };
    snapshot.rx_activity = onboard_autonomy::application::LinkActivity{
        .sequence = 2,
        .observed_at = std::chrono::milliseconds(1200),
        .message_name =
            "OPEN_DRONE_ID_MESSAGE_PACK_REGISTRATION",
        .detail = "RECEIVED",
    };

    const auto output = onboard_autonomy::presentation::console::render_console(
        snapshot,
        "serial:///dev/ttyACM0?baud=115200",
        false
    );

    require(
        output.find("OPEN_DRONE_ID_MESSAGE_PACK_REG...") !=
            std::string::npos,
        "long real MAVLink names must be clipped without overflowing "
        "the fixed-width wire"
    );
}

void camera_pipeline_metrics_are_visible() {
    onboard_autonomy::application::AppSnapshot snapshot;
    snapshot.camera = onboard_autonomy::application::CameraSnapshot{
        .phase =
            onboard_autonomy::application::ports::
                CameraSourcePhase::streaming,
        .source = "rpicam-vid camera 0",
        .error = "",
        .width = 640,
        .height = 480,
        .received_frames = 120,
        .camera_restarts = 2,
        .frames_with_capture_timestamp = 120,
        .measured_fps = 30.01,
        .latest_latency_ms = 42.1,
        .average_latency_ms = 39.5,
        .maximum_latency_ms = 51.2,
        .latest_frame_age_ms = std::nullopt,
    };

    const auto output =
        onboard_autonomy::presentation::console::render_console(
            snapshot,
            "serial:///dev/ttyACM0?baud=115200",
            false
        );
    require(
        output.find("CAMERA STREAMING") != std::string::npos &&
            output.find("640x480 YUV420") != std::string::npos &&
            output.find("30.0 FPS") != std::string::npos &&
            output.find("2 RESTARTS") != std::string::npos,
        "console must show the live camera stream"
    );
    require(
        output.find("CAMERA LATENCY 42.1 MS LATEST") !=
                std::string::npos &&
            output.find("39.5 MS AVG") != std::string::npos &&
            output.find("DROP 0") != std::string::npos,
        "console must show camera latency and dropped frames"
    );
}

void vision_pipeline_and_target_are_visible() {
    onboard_autonomy::application::AppSnapshot snapshot;
    snapshot.vision = onboard_autonomy::application::VisionSnapshot{
        .detector = "AprilTag 3 / tagStandard41h12",
        .processed_frames = 42,
        .frames_with_targets = 3,
        .total_targets = 3,
        .latest_processing_ms = 5.2,
        .average_processing_ms = 4.8,
        .maximum_processing_ms = 6.1,
        .last_detection_age_ms = 0.0,
        .latest_targets =
            {
                {
                    .id = 0,
                    .family = "tagStandard41h12",
                    .center = {.x_px = 319.5, .y_px = 239.5},
                    .corners = {},
                    .corrected_bits = 0,
                    .decision_margin = 88.4,
                    .pose =
                        onboard_autonomy::domain::TargetPose{
                            .position =
                                {
                                    .right_m = 0.12,
                                    .down_m = -0.05,
                                    .forward_m = 1.42,
                                },
                        },
                },
            },
        .target_track =
            {
                .phase =
                    onboard_autonomy::application::
                        TargetTrackPhase::tracking,
                .target_id = 0,
                .consecutive_observations = 3,
                .required_observations = 3,
                .accepted_observations = 3,
                .observation_age_ms = 8.0,
                .latest_decision_margin = 88.4,
                .position =
                    onboard_autonomy::domain::CameraFramePosition{
                        .right_m = 0.12,
                        .down_m = -0.05,
                        .forward_m = 1.42,
                    },
            },
    };

    const auto output =
        onboard_autonomy::presentation::console::render_console(
            snapshot,
            "serial:///dev/ttyACM0?baud=115200",
            false
        );
    require(
        output.find("VISION AprilTag 3 / tagStandard41h12") !=
                std::string::npos &&
            output.find("4.8 MS AVG") != std::string::npos,
        "console must show the active vision detector"
    );
    require(
        output.find("TARGET ID 0   |   TRACKING") !=
            std::string::npos,
        "console must show the confirmed target track"
    );
    require(
        output.find(
            "X RIGHT 0.12 M   |   Y DOWN -0.05 M   |   "
            "Z FORWARD 1.42 M"
        ) != std::string::npos,
        "metric target pose must be explicit in camera coordinates"
    );
}

}  // namespace

void run_console_view_tests() {
    healthy_snapshot_is_human_readable();
    disconnected_snapshot_is_waiting();
    command_bus_shows_both_directions();
    colors_group_related_elements();
    long_mavlink_message_names_are_safely_clipped();
    camera_pipeline_metrics_are_visible();
    vision_pipeline_and_target_are_visible();
}
