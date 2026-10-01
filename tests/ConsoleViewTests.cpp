#include "TestCases.hpp"

#include "onboard_autonomy/operator/ui/screen/BoardTypeCatalog.hpp"
#include "onboard_autonomy/operator/ui/screen/ConsoleSnapshotSink.hpp"
#include "onboard_autonomy/operator/ui/screen/ConsoleView.hpp"

#include <chrono>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

namespace ui = onboard_autonomy::operator_interface::ui;
namespace mission = onboard_autonomy::mission;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void healthy_snapshot_is_human_readable() {
    mission::AppSnapshot app;
    auto& vehicle = app.vehicle;
    vehicle.connected = true;
    vehicle.gps_ready = true;
    vehicle.battery_ready = true;
    vehicle.system_health_known = true;
    vehicle.system_health_ok = true;
    vehicle.armable = true;
    vehicle.gps_fix_type = 6;
    vehicle.satellites_visible = 10;
    vehicle.battery_voltage_v = 12.6;
    vehicle.battery_remaining_pct = 100;
    vehicle.autopilot_metadata = mission::AutopilotMetadata{
        .firmware_major = 4,
        .firmware_minor = 6,
        .firmware_patch = 3,
        .firmware_release_type = 255,
        .board_version = (56U << 16U) | 2U,
    };
    app.companion_heartbeat_active = true;
    app.companion_link_failsafe.phase =
        mission::CompanionLinkFailsafePhase::accepted;
    app.companion_link_failsafe.action =
        mission::ArduPilotGcsFailsafeAction::land;
    app.companion_link_failsafe.timeout_s = 3.0;
    app.companion_link_failsafe.configured_gcs_system_id = 1;
    app.telemetry.state = mission::TelemetrySetupState::active;
    app.telemetry.completed_requests = 6;
    app.telemetry.total_requests = 6;

    std::istringstream table{"Reserved \"PX4 [BL] FMU v6C.x\" 56\n"};
    const auto catalog = ui::BoardTypeCatalog::from_stream(table);
    const auto output =
        ui::render_console(app, "udp://127.0.0.1:14550", {}, &catalog);
    for (const auto* expected : {"[ READY ]",
             "GPS RTK FIXED / 10 SAT",
             "BAT 12.60 V / 100%",
             "TELEMETRY READY / 6 STREAMS",
             "FIRMWARE 4.6.3 OFFICIAL",
             "PX4 [BL] FMU v6C.x / ID 56 / SILICON 2",
             "NO ACTIVE WARNINGS",
             "LINK FAILSAFE READY / ARDUPILOT LAND / 3.0 S / SYSID 1"}) {
        require(output.find(expected) != std::string::npos,
            std::string("missing operator information: ") + expected);
    }
    require(output.find("RASPBERRY PI 5") == std::string::npos,
        "configured companion hardware must not be presented as detected "
        "hardware");
    require(output.find("CAMERA / VISION") == std::string::npos,
        "unused camera sections must not consume screen space");
}

void disconnected_snapshot_is_waiting() {
    const auto output = ui::render_console({}, "udp://127.0.0.1:14550");
    require(output.find("WAITING FOR FLIGHT CONTROLLER") != std::string::npos &&
                output.find("ARM STATE UNKNOWN") != std::string::npos,
        "a disconnected controller must not look ready or disarmed");
}

void link_activity_keeps_the_last_frame_and_its_age() {
    mission::AppSnapshot app;
    app.elapsed = std::chrono::milliseconds(1800);
    app.tx_activity = mission::LinkActivity{
        .observed_at = std::chrono::milliseconds(1100),
        .message_name = "COMMAND_LONG",
        .detail = "SET_MODE",
    };
    app.rx_activity = mission::LinkActivity{
        .observed_at = std::chrono::milliseconds(1100),
        .message_name = "COMMAND_ACK",
        .detail = "SET_MODE ACCEPTED",
    };
    const auto output = ui::render_console(app, "udp://127.0.0.1:14550");
    require(
        output.find("TX  COMMAND_LONG: SET_MODE  / 0.7 S AGO") !=
                std::string::npos &&
            output.find("RX  COMMAND_ACK: SET_MODE ACCEPTED  / 0.7 S AGO") !=
                std::string::npos,
        "last traffic must remain readable with its age instead of blinking "
        "and disappearing");
    app.elapsed = std::chrono::milliseconds(900);
    require(
        ui::render_console(app, "udp://127.0.0.1:14550").find("AGE UNKNOWN") !=
            std::string::npos,
        "a future frame timestamp must not produce a negative age");
}

void shortcuts_reflect_the_actual_input_mode() {
    mission::AppSnapshot app;
    app.motion_commands_allowed = true;
    app.aerial_tracking_available = true;
    const auto passive = ui::render_console(app, "fake://transport");
    require(passive.find("[2]") == std::string::npos &&
                passive.find("[R]") == std::string::npos &&
                passive.find("[Q]") == std::string::npos &&
                passive.find("Ctrl+C exit") != std::string::npos,
        "a noninteractive console must not advertise inactive keyboard "
        "shortcuts");
    const ui::ConsoleViewOptions interactive{.interactive_input = true};
    const auto active =
        ui::render_console(app, "fake://transport", interactive);
    require(active.find("[2] TRACK AIRBORNE TARGET (HOLD + YAW)") !=
                    std::string::npos &&
                active.find("[R] ABORT MISSION + RTL") != std::string::npos &&
                active.find("[Q] QUIT") != std::string::npos,
        "existing interactive shortcuts must remain discoverable");
    app.motion_commands_allowed = false;
    const auto observation =
        ui::render_console(app, "fake://transport", interactive);
    require(observation.find("[Q] QUIT") != std::string::npos &&
                observation.find("[2]") == std::string::npos &&
                observation.find("[R]") == std::string::npos,
        "observation mode must expose Quit without advertising motion "
        "controls");
}

void all_warnings_and_long_protocol_text_remain_readable() {
    mission::AppSnapshot app;
    app.vehicle.warnings = {"Battery below arming threshold",
        "GPS not ready",
        std::string(170, 'w')};
    app.tx_activity = mission::LinkActivity{
        .message_name = "OPEN_DRONE_ID_MESSAGE_PACK_REGISTRATION",
        .detail = std::string(180, 'd'),
    };
    const auto output = ui::render_console(app, std::string(140, 'e'));
    require(output.find("Battery below arming threshold") !=
                    std::string::npos &&
                output.find("GPS not ready") != std::string::npos,
        "every active warning must be displayed");
    require(output.find("OPEN_DRONE_ID_MESSAGE_PACK_REGISTRATION") !=
                std::string::npos,
        "real MAVLink names must not be hidden by a decorative wire");
    std::istringstream lines{output};
    std::string line;
    while (std::getline(lines, line)) {
        require(line.size() == 80,
            "wrapped text must stay inside an 80-column screen");
    }
    app.vehicle.warnings = {"Bad\x1b[2J\nmessage\ttext"};
    const auto untrusted = ui::render_console(app, "fake://transport");
    require(untrusted.find('\x1b') == std::string::npos &&
                untrusted.find('\t') == std::string::npos,
        "protocol text must not inject terminal control sequences");
}

void camera_and_vision_metrics_remain_visible() {
    mission::AppSnapshot app;
    app.camera = mission::CameraSnapshot{
        .phase = mission::ports::CameraSourcePhase::streaming,
        .width = 640,
        .height = 480,
        .received_frames = 120,
        .camera_restarts = 2,
        .measured_fps = 30.01,
        .latest_latency_ms = 42.1,
        .average_latency_ms = 39.5,
    };
    app.vision = mission::VisionSnapshot{
        .detector = "synthetic detector",
        .processed_frames = 42,
        .average_processing_ms = 4.8,
        .latest_targets = {{
            .family = "object",
            .center = {.x_px = 319.5, .y_px = 239.5},
            .confidence_percent = 88.4,
        }},
    };
    const auto output = ui::render_console(app, "fake://transport");
    for (const auto* expected : {"CAMERA STREAMING",
             "640x480 YUV420",
             "30.0 FPS",
             "2 RESTARTS",
             "CAMERA LATENCY 42.1 MS LATEST",
             "39.5 MS AVG",
             "DROP 0",
             "VISION synthetic detector",
             "4.8 MS AVG",
             "CENTER 319.5/239.5 PX",
             "CONFIDENCE 88.4%"}) {
        require(output.find(expected) != std::string::npos,
            std::string("missing camera diagnostic: ") + expected);
    }
    require(output.find("TAG") == std::string::npos &&
                output.find("X RIGHT") == std::string::npos,
        "the removed marker model must not return in the console");
}

void plain_sink_never_emits_terminal_controls() {
    std::ostringstream output;
    {
        ui::ConsoleSnapshotSink sink{output,
            "fake://transport",
            nullptr,
            {.use_color = true}};
        sink.consume({}, {});
        sink.consume({}, {});
    }
    require(output.str().find('\x1b') == std::string::npos,
        "redirected snapshots must contain neither cursor controls nor colors, "
        "even on destruction");
    require(output.str().find("\n\n+") != std::string::npos,
        "plain snapshots must remain separate complete records");
}

void terminal_sink_clears_a_shorter_frame_and_restores_cursor() {
    std::ostringstream output;
    mission::AppSnapshot app;
    app.camera = mission::CameraSnapshot{};
    {
        ui::ConsoleSnapshotSink sink{output,
            "fake://transport",
            nullptr,
            {},
            ui::ConsoleOutputMode::terminal};
        sink.consume(app, {});
        const auto first_frame = output.str().size();
        app.camera.reset();
        sink.consume(app, {});
        const auto second_frame = output.str().substr(first_frame);
        require(second_frame.starts_with("\x1b[H") &&
                    second_frame.ends_with("\x1b[J"),
            "redraw must erase old rows after a camera section disappears");
        require(second_frame.find("CAMERA / VISION") == std::string::npos,
            "the shorter frame must not retain the removed camera section");
    }
    require(output.str().find("\x1b[?25l") != std::string::npos &&
                output.str().ends_with("\x1b[?25h\x1b[0m\n"),
        "terminal lifetime must restore the hidden cursor");
}

} // namespace

void run_console_view_tests() {
    healthy_snapshot_is_human_readable();
    disconnected_snapshot_is_waiting();
    link_activity_keeps_the_last_frame_and_its_age();
    shortcuts_reflect_the_actual_input_mode();
    all_warnings_and_long_protocol_text_remain_readable();
    camera_and_vision_metrics_remain_visible();
    plain_sink_never_emits_terminal_controls();
    terminal_sink_clears_a_shorter_frame_and_restores_cursor();
}
