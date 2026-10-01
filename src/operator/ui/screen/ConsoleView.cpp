#include "onboard_autonomy/operator/ui/screen/ConsoleView.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace onboard_autonomy::operator_interface::ui {
namespace {

constexpr std::size_t kConsoleWidth = 80;
constexpr std::size_t kInnerWidth = kConsoleWidth - 2;
constexpr std::size_t kContentWidth = kInnerWidth - 2;
constexpr unsigned char kDeleteCharacter = 127;
constexpr std::uint8_t kGpsUnavailable = 0;
constexpr std::uint8_t kGpsNoFix = 1;
constexpr std::uint8_t kGps2dFix = 2;
constexpr std::uint8_t kGps3dFix = 3;
constexpr std::uint8_t kGpsDgpsFix = 4;
constexpr std::uint8_t kGpsRtkFloat = 5;
constexpr std::uint8_t kGpsRtkFixed = 6;
constexpr std::uint8_t kGpsStaticFixed = 7;
constexpr std::uint8_t kGpsPppFix = 8;
constexpr std::uint32_t kStabilizeMode = 0;
constexpr std::uint32_t kAutoMode = 3;
constexpr std::uint32_t kGuidedMode = 4;
constexpr std::uint32_t kLoiterMode = 5;
constexpr std::uint32_t kReturnToLaunchMode = 6;
constexpr std::uint32_t kLandMode = 9;
constexpr std::uint32_t kPositionHoldMode = 16;
constexpr std::uint8_t kArduPilotAutopilotType = 3;
constexpr std::uint8_t kOfficialReleaseType = 255;
constexpr std::uint8_t kReleaseCandidateTypeStart = 192;
constexpr std::uint8_t kBetaReleaseTypeStart = 128;
constexpr std::uint8_t kAlphaReleaseTypeStart = 64;
constexpr std::uint32_t kBoardTypeBitShift = 16;

enum class Tone {
    normal,
    good,
    waiting,
    bad,
    accent,
    chrome,
    dim,
};

std::string_view ansi_code(const Tone tone) {
    switch (tone) {
    case Tone::normal:
        return "\x1b[97m";
    case Tone::good:
        return "\x1b[92m";
    case Tone::waiting:
        return "\x1b[93m";
    case Tone::bad:
        return "\x1b[91m";
    case Tone::accent:
        return "\x1b[96m";
    case Tone::chrome:
        return "\x1b[94m";
    case Tone::dim:
        return "\x1b[90m";
    }
    return "\x1b[0m";
}

std::string paint(std::string value, const Tone tone, const bool use_color) {
    if (!use_color) {
        return value;
    }
    return std::string(ansi_code(tone)) + value + "\x1b[0m";
}

// Protocol text is untrusted terminal content. Keep printable bytes only.
std::string printable(const std::string_view value) {
    std::string result;
    result.reserve(value.size());
    for (const unsigned char character : value) {
        result += character >= ' ' && character != kDeleteCharacter
                      ? static_cast<char>(character)
                      : ' ';
    }
    return result;
}

std::string fitted(const std::string_view value, const std::size_t width) {
    std::string result{value.substr(0, width)};
    result.append(width - result.size(), ' ');
    return result;
}

void write_border(std::ostringstream& output,
    const std::string_view title,
    const bool use_color) {
    const auto label =
        title.empty() ? std::string{} : "-- " + std::string(title) + " ";
    output << paint("+" + label + std::string(kInnerWidth - label.size(), '-') +
                        "+",
                  Tone::chrome,
                  use_color)
           << '\n';
}

void write_line(std::ostringstream& output,
    const std::string_view value,
    const Tone tone,
    const bool use_color) {
    const auto text = printable(value);
    std::string_view remaining{text};
    while (!remaining.empty()) {
        auto count = std::min(remaining.size(), kContentWidth);
        if (remaining.size() > kContentWidth) {
            const auto space = remaining.rfind(' ', kContentWidth);
            if (space != std::string_view::npos && space > 0) {
                count = space;
            }
        }
        output << paint("| ", Tone::chrome, use_color)
               << paint(fitted(remaining.substr(0, count), kContentWidth),
                      tone,
                      use_color)
               << paint(" |", Tone::chrome, use_color) << '\n';
        remaining.remove_prefix(count);
        while (!remaining.empty() && remaining.front() == ' ') {
            remaining.remove_prefix(1);
        }
    }
}

std::string gps_fix_name(const std::optional<std::uint8_t> fix_type) {
    if (!fix_type.has_value()) {
        return "WAITING";
    }

    switch (*fix_type) {
    case kGpsUnavailable:
        return "UNAVAILABLE";
    case kGpsNoFix:
        return "NO FIX";
    case kGps2dFix:
        return "2D";
    case kGps3dFix:
        return "3D";
    case kGpsDgpsFix:
        return "DGPS";
    case kGpsRtkFloat:
        return "RTK FLOAT";
    case kGpsRtkFixed:
        return "RTK FIXED";
    case kGpsStaticFixed:
        return "STATIC FIXED";
    case kGpsPppFix:
        return "PPP";
    default:
        return "UNKNOWN";
    }
}

std::string flight_mode_name(const std::optional<std::uint32_t> mode) {
    if (!mode.has_value()) {
        return "UNKNOWN";
    }

    switch (*mode) {
    case kStabilizeMode:
        return "STABILIZE";
    case kAutoMode:
        return "AUTO";
    case kGuidedMode:
        return "GUIDED";
    case kLoiterMode:
        return "LOITER";
    case kReturnToLaunchMode:
        return "RTL";
    case kLandMode:
        return "LAND";
    case kPositionHoldMode:
        return "POSITION HOLD";
    default:
        return "MODE " + std::to_string(*mode);
    }
}

std::string autopilot_name(const std::optional<std::uint8_t> type) {
    if (!type.has_value()) {
        return "WAITING";
    }
    if (*type == kArduPilotAutopilotType) {
        return "ARDUPILOT";
    }
    return "AUTOPILOT " + std::to_string(*type);
}

std::string firmware_release_name(const std::uint8_t release_type) {
    if (release_type == kOfficialReleaseType) {
        return "OFFICIAL";
    }
    if (release_type >= kReleaseCandidateTypeStart) {
        return "RC";
    }
    if (release_type >= kBetaReleaseTypeStart) {
        return "BETA";
    }
    if (release_type >= kAlphaReleaseTypeStart) {
        return "ALPHA";
    }
    return "DEV";
}

std::string firmware_detail(const mission::VehicleSnapshot& vehicle) {
    if (!vehicle.connected) {
        return "FIRMWARE WAITING";
    }
    if (!vehicle.autopilot_metadata.has_value()) {
        return "FIRMWARE REQUESTING";
    }

    const auto& metadata = *vehicle.autopilot_metadata;
    return "FIRMWARE " + std::to_string(metadata.firmware_major) + "." +
           std::to_string(metadata.firmware_minor) + "." +
           std::to_string(metadata.firmware_patch) + " " +
           firmware_release_name(metadata.firmware_release_type);
}

std::uint16_t board_type_id(const std::uint32_t board_version) {
    return static_cast<std::uint16_t>(board_version >> kBoardTypeBitShift);
}

std::optional<BoardTypeMatch> resolve_board_type(
    const std::uint32_t board_version,
    const BoardTypeResolver* resolver) {
    if (resolver == nullptr) {
        return std::nullopt;
    }
    return resolver->resolve(board_type_id(board_version));
}

std::string board_type_name(const std::uint32_t board_version,
    const BoardTypeResolver* resolver) {
    if (const auto match = resolve_board_type(board_version, resolver);
        match.has_value()) {
        return match->preferred_name;
    }
    return "BOARD TYPE " + std::to_string(board_type_id(board_version));
}

std::string board_detail(const mission::VehicleSnapshot& vehicle,
    const BoardTypeResolver* resolver) {
    if (!vehicle.connected || !vehicle.autopilot_metadata.has_value()) {
        return "BOARD WAITING";
    }

    const auto board_version = vehicle.autopilot_metadata->board_version;
    if (board_version == 0U) {
        return "BOARD UNREPORTED";
    }

    const auto silicon_id = board_version & 0xFFU;
    std::string result = board_type_name(board_version, resolver) + " / ID " +
                         std::to_string(board_type_id(board_version)) +
                         " / SILICON " + std::to_string(silicon_id);
    if (const auto match = resolve_board_type(board_version, resolver);
        match.has_value() && match->aliases.size() > 1) {
        result += " / " + std::to_string(match->aliases.size()) + " ALIASES";
    }
    return result;
}

std::string telemetry_detail(const mission::TelemetryStatus& telemetry) {
    const std::string progress = std::to_string(telemetry.completed_requests) +
                                 "/" + std::to_string(telemetry.total_requests);

    switch (telemetry.state) {
    case mission::TelemetrySetupState::waiting_for_vehicle:
        return "TELEMETRY WAITING";
    case mission::TelemetrySetupState::configuring:
        return "TELEMETRY SETUP " + progress + " ACCEPTED";
    case mission::TelemetrySetupState::active:
        return "TELEMETRY READY / " + std::to_string(telemetry.total_requests) +
               " STREAMS";
    case mission::TelemetrySetupState::failed:
        return "TELEMETRY FAILED / " + progress + " ACCEPTED";
    }
    return "TELEMETRY UNKNOWN";
}

std::string companion_link_failsafe_detail(
    const mission::CompanionLinkFailsafeSnapshot& failsafe) {
    if (!failsafe.accepted()) {
        return "LINK FAILSAFE / " + failsafe.detail;
    }

    std::ostringstream detail;
    detail << "LINK FAILSAFE READY / ARDUPILOT LAND";
    if (failsafe.timeout_s.has_value()) {
        detail << " / " << std::fixed << std::setprecision(1)
               << *failsafe.timeout_s << " S";
    }
    if (failsafe.configured_gcs_system_id.has_value()) {
        detail << " / SYSID "
               << static_cast<unsigned int>(*failsafe.configured_gcs_system_id);
    }
    return detail.str();
}

Tone companion_link_failsafe_tone(
    const mission::CompanionLinkFailsafeSnapshot& failsafe) {
    switch (failsafe.phase) {
    case mission::CompanionLinkFailsafePhase::accepted:
        return Tone::good;
    case mission::CompanionLinkFailsafePhase::rejected:
        return Tone::bad;
    case mission::CompanionLinkFailsafePhase::reading_parameters:
        return Tone::waiting;
    case mission::CompanionLinkFailsafePhase::waiting_for_vehicle:
        return Tone::dim;
    }
    return Tone::bad;
}

Tone metadata_tone(const mission::AppSnapshot& snapshot) {
    if (snapshot.telemetry.state == mission::TelemetrySetupState::failed) {
        return Tone::bad;
    }
    if (snapshot.telemetry.state == mission::TelemetrySetupState::active &&
        snapshot.vehicle.autopilot_metadata.has_value()) {
        return Tone::good;
    }
    return snapshot.vehicle.connected ? Tone::waiting : Tone::dim;
}

std::string camera_phase_name(const mission::CameraSnapshot& camera) {
    switch (camera.phase) {
    case mission::ports::CameraSourcePhase::starting:
        return "CAMERA STARTING";
    case mission::ports::CameraSourcePhase::streaming:
        return "CAMERA STREAMING";
    case mission::ports::CameraSourcePhase::reconnecting:
        return "CAMERA RECONNECTING";
    case mission::ports::CameraSourcePhase::stopped:
        return "CAMERA STOPPED";
    case mission::ports::CameraSourcePhase::failed:
        return "CAMERA FAILED";
    }
    return "CAMERA FAILED";
}

Tone camera_tone(const mission::CameraSnapshot& camera) {
    switch (camera.phase) {
    case mission::ports::CameraSourcePhase::starting:
        return Tone::waiting;
    case mission::ports::CameraSourcePhase::streaming:
        return Tone::good;
    case mission::ports::CameraSourcePhase::reconnecting:
        return Tone::waiting;
    case mission::ports::CameraSourcePhase::stopped:
        return Tone::dim;
    case mission::ports::CameraSourcePhase::failed:
        return Tone::bad;
    }
    return Tone::bad;
}

std::string camera_stream_detail(const mission::CameraSnapshot& camera) {
    std::ostringstream detail;
    detail << camera_phase_name(camera);
    if (camera.width > 0U && camera.height > 0U) {
        detail << "   |   " << camera.width << "x" << camera.height
               << " YUV420";
    }
    if (camera.measured_fps.has_value()) {
        detail << "   |   " << std::fixed << std::setprecision(1)
               << *camera.measured_fps << " FPS";
    }
    detail << "   |   " << camera.received_frames << " FRAMES";
    if (camera.camera_restarts > 0U) {
        detail << "   |   " << camera.camera_restarts << " RESTARTS";
    }
    return detail.str();
}

std::string camera_latency_detail(const mission::CameraSnapshot& camera) {
    if (!camera.error.empty()) {
        return camera.error;
    }
    if (!camera.latest_latency_ms.has_value()) {
        return "WAITING FOR FRAMEWALLCLOCK METADATA";
    }

    std::ostringstream detail;
    detail << std::fixed << std::setprecision(1) << "CAMERA LATENCY "
           << *camera.latest_latency_ms << " MS LATEST";
    if (camera.average_latency_ms.has_value()) {
        detail << " / " << *camera.average_latency_ms << " MS AVG";
    }
    if (camera.maximum_latency_ms.has_value()) {
        detail << " / " << *camera.maximum_latency_ms << " MS MAX";
    }
    detail << "   |   DROP " << camera.dropped_before_processing;
    return detail.str();
}

std::string vision_pipeline_detail(const mission::VisionSnapshot& vision) {
    std::ostringstream detail;
    detail << "VISION " << vision.detector << "   |   "
           << vision.processed_frames << " FRAMES";
    if (vision.average_processing_ms.has_value()) {
        detail << "   |   " << std::fixed << std::setprecision(1)
               << *vision.average_processing_ms << " MS AVG";
    }
    return detail.str();
}

std::string vision_target_detail(const mission::VisionSnapshot& vision) {
    if (vision.latest_targets.empty()) {
        return "NO OBJECT DETECTED";
    }
    const auto& target = vision.latest_targets.front();
    std::ostringstream detail;
    detail << target.family << "   |   CENTER " << std::fixed
           << std::setprecision(1) << target.center.x_px << "/"
           << target.center.y_px << " PX   |   CONFIDENCE "
           << target.confidence_percent << "%";
    if (vision.latest_targets.size() > 1U) {
        detail << "   |   " << vision.latest_targets.size() << " OBJECTS";
    }
    return detail.str();
}

Tone vision_target_tone(const mission::VisionSnapshot& vision) {
    return vision.latest_targets.empty() ? Tone::dim : Tone::good;
}

std::string altitude_detail(const mission::VehicleSnapshot& vehicle) {
    if (!vehicle.relative_altitude_m.has_value()) {
        return "--";
    }

    std::ostringstream altitude;
    altitude << std::fixed << std::setprecision(2)
             << *vehicle.relative_altitude_m << " M";
    return altitude.str();
}

std::string gps_detail(const mission::VehicleSnapshot& vehicle) {
    std::string result = gps_fix_name(vehicle.gps_fix_type);
    if (vehicle.satellites_visible.has_value()) {
        result += " / " + std::to_string(*vehicle.satellites_visible) + " SAT";
    }
    return result;
}

std::string battery_detail(const mission::VehicleSnapshot& vehicle) {
    std::vector<std::string> parts;
    if (vehicle.battery_voltage_v.has_value()) {
        std::ostringstream voltage;
        voltage << std::fixed << std::setprecision(2)
                << *vehicle.battery_voltage_v << " V";
        parts.push_back(voltage.str());
    }
    if (vehicle.battery_remaining_pct.has_value()) {
        parts.push_back(std::to_string(*vehicle.battery_remaining_pct) + "%");
    }
    if (parts.empty()) {
        return "WAITING";
    }

    std::ostringstream output;
    for (std::size_t index = 0; index < parts.size(); ++index) {
        if (index > 0) {
            output << " / ";
        }
        output << parts[index];
    }
    return output.str();
}

std::string startup_phase_name(const mission::FlightStartupPhase phase) {
    switch (phase) {
    case mission::FlightStartupPhase::disabled:
    case mission::FlightStartupPhase::idle:
        return "IDLE";
    case mission::FlightStartupPhase::waiting_for_vehicle:
    case mission::FlightStartupPhase::waiting_for_readiness:
        return "WAITING";
    case mission::FlightStartupPhase::setting_guided:
        return "GUIDED";
    case mission::FlightStartupPhase::arming:
        return "ARMING";
    case mission::FlightStartupPhase::taking_off:
        return "TAKEOFF";
    case mission::FlightStartupPhase::completed:
        return "COMPLETE";
    case mission::FlightStartupPhase::failed:
        return "FAILED";
    }
    return "UNKNOWN";
}

std::string autonomy_phase_name(const mission::AutonomyRuntimePhase phase) {
    switch (phase) {
    case mission::AutonomyRuntimePhase::disabled:
    case mission::AutonomyRuntimePhase::idle:
        return "IDLE";
    case mission::AutonomyRuntimePhase::waiting_for_startup:
        return "WAITING";
    case mission::AutonomyRuntimePhase::active:
        return "ACTIVE";
    case mission::AutonomyRuntimePhase::suspended:
        return "SUSPENDED";
    case mission::AutonomyRuntimePhase::returning_to_launch:
        return "RTL";
    case mission::AutonomyRuntimePhase::completed:
        return "COMPLETE";
    case mission::AutonomyRuntimePhase::failed:
        return "FAILED";
    }
    return "UNKNOWN";
}

Tone autonomy_tone(const mission::AutonomyRuntimePhase phase) {
    switch (phase) {
    case mission::AutonomyRuntimePhase::completed:
        return Tone::good;
    case mission::AutonomyRuntimePhase::failed:
        return Tone::bad;
    case mission::AutonomyRuntimePhase::active:
    case mission::AutonomyRuntimePhase::returning_to_launch:
        return Tone::accent;
    case mission::AutonomyRuntimePhase::disabled:
    case mission::AutonomyRuntimePhase::idle:
        return Tone::dim;
    case mission::AutonomyRuntimePhase::waiting_for_startup:
    case mission::AutonomyRuntimePhase::suspended:
        return Tone::waiting;
    }
    return Tone::normal;
}

std::string activity_detail(const std::string_view direction,
    const std::optional<mission::LinkActivity>& activity,
    const std::chrono::milliseconds elapsed) {
    if (!activity.has_value()) {
        return std::string(direction) + "  WAITING FOR FIRST FRAME";
    }
    std::ostringstream detail;
    detail << direction << "  " << activity->message_name;
    if (!activity->detail.empty()) {
        detail << ": " << activity->detail;
    }
    if (elapsed >= activity->observed_at) {
        const auto age =
            std::chrono::duration<double>(elapsed - activity->observed_at);
        detail << "  / " << std::fixed << std::setprecision(1) << age.count()
               << " S AGO";
    } else {
        detail << "  / AGE UNKNOWN";
    }
    return detail.str();
}

std::string overall_status(const mission::AppSnapshot& snapshot) {
    const auto& vehicle = snapshot.vehicle;
    const auto phase = snapshot.autonomy.phase;
    const bool telemetry_complete = vehicle.gps_fix_type.has_value() &&
                                    vehicle.battery_voltage_v.has_value() &&
                                    vehicle.system_health_known;

    if (phase == mission::AutonomyRuntimePhase::completed) {
        return "FLIGHT COMPLETE";
    }
    if (phase == mission::AutonomyRuntimePhase::failed ||
        snapshot.flight_startup.phase == mission::FlightStartupPhase::failed) {
        return "FLIGHT FAILED";
    }
    if (phase == mission::AutonomyRuntimePhase::returning_to_launch) {
        return "RETURNING TO LAUNCH";
    }
    if (phase != mission::AutonomyRuntimePhase::disabled &&
        phase != mission::AutonomyRuntimePhase::idle) {
        return "AUTONOMY RUNNING";
    }
    if (!vehicle.connected) {
        return "WAITING FOR FLIGHT CONTROLLER";
    }
    if (!telemetry_complete) {
        return "CHECKING TELEMETRY";
    }
    return vehicle.armable ? "READY" : "NOT READY";
}

Tone overall_tone(const mission::AppSnapshot& snapshot) {
    const std::string status = overall_status(snapshot);
    if (status == "READY" || status == "FLIGHT COMPLETE") {
        return Tone::good;
    }
    if (status == "NOT READY" || status == "FLIGHT FAILED") {
        return Tone::bad;
    }
    if (status == "AUTONOMY RUNNING" || status == "RETURNING TO LAUNCH") {
        return Tone::accent;
    }
    return Tone::waiting;
}

void write_vehicle(std::ostringstream& output,
    const mission::VehicleSnapshot& vehicle,
    const BoardTypeResolver* resolver,
    const bool use_color) {
    write_border(output, "VEHICLE", use_color);
    const auto tone = vehicle.connected ? Tone::normal : Tone::dim;
    write_line(output,
        "MODE " + flight_mode_name(vehicle.flight_mode) + " / " +
            (vehicle.connected ? (vehicle.armed ? "ARMED" : "DISARMED")
                               : "ARM STATE UNKNOWN") +
            "   |   ALT " + altitude_detail(vehicle),
        tone,
        use_color);
    write_line(output,
        "GPS " + gps_detail(vehicle) + "   |   BAT " + battery_detail(vehicle),
        tone,
        use_color);
    write_line(output,
        "FC " + autopilot_name(vehicle.autopilot_type) + "   |   " +
            firmware_detail(vehicle),
        tone,
        use_color);
    write_line(output, board_detail(vehicle, resolver), tone, use_color);
}

void write_health(std::ostringstream& output,
    const mission::AppSnapshot& snapshot,
    const bool use_color) {
    write_border(output, "HEALTH", use_color);
    write_line(output,
        telemetry_detail(snapshot.telemetry) + "   |   COMPANION HEARTBEAT " +
            (snapshot.companion_heartbeat_active ? "ON" : "WAITING"),
        metadata_tone(snapshot),
        use_color);
    write_line(output,
        companion_link_failsafe_detail(snapshot.companion_link_failsafe),
        companion_link_failsafe_tone(snapshot.companion_link_failsafe),
        use_color);
    if (snapshot.vehicle.warnings.empty()) {
        write_line(output,
            snapshot.vehicle.connected ? "NO ACTIVE WARNINGS"
                                       : "WARNINGS AVAILABLE AFTER CONNECTION",
            snapshot.vehicle.connected ? Tone::good : Tone::dim,
            use_color);
    }
    for (const auto& warning : snapshot.vehicle.warnings) {
        write_line(output, "! " + warning, Tone::bad, use_color);
    }
}

void write_camera_and_vision(std::ostringstream& output,
    const mission::AppSnapshot& snapshot,
    const bool use_color) {
    if (!snapshot.camera.has_value() && !snapshot.vision.has_value()) {
        return;
    }
    write_border(output, "CAMERA / VISION", use_color);
    if (snapshot.camera.has_value()) {
        const auto tone = camera_tone(*snapshot.camera);
        write_line(output,
            camera_stream_detail(*snapshot.camera),
            tone,
            use_color);
        write_line(output,
            camera_latency_detail(*snapshot.camera),
            tone,
            use_color);
    }
    if (snapshot.vision.has_value()) {
        write_line(output,
            vision_pipeline_detail(*snapshot.vision),
            Tone::accent,
            use_color);
        write_line(output,
            vision_target_detail(*snapshot.vision),
            vision_target_tone(*snapshot.vision),
            use_color);
    }
}

void write_session(std::ostringstream& output,
    const mission::AppSnapshot& snapshot,
    const bool use_color) {
    const auto& startup = snapshot.flight_startup;
    const auto& autonomy = snapshot.autonomy;
    if (startup.phase == mission::FlightStartupPhase::disabled &&
        autonomy.phase == mission::AutonomyRuntimePhase::disabled) {
        return;
    }
    const bool startup_finished =
        startup.phase == mission::FlightStartupPhase::completed ||
        startup.phase == mission::FlightStartupPhase::idle ||
        startup.phase == mission::FlightStartupPhase::disabled;
    write_border(output, "SESSION", use_color);
    write_line(output,
        "AUTONOMY: " + autonomy_phase_name(autonomy.phase) +
            "   |   STARTUP: " + startup_phase_name(startup.phase),
        autonomy_tone(autonomy.phase),
        use_color);
    write_line(output,
        startup_finished ? autonomy.detail : startup.detail,
        autonomy_tone(autonomy.phase),
        use_color);
}

void write_controls(std::ostringstream& output,
    const mission::AppSnapshot& snapshot,
    const ConsoleViewOptions& options) {
    write_border(output, "CONTROLS", options.use_color);
    if (!options.interactive_input) {
        write_line(output,
            "LIVE VIEW   |   Ctrl+C exit",
            Tone::dim,
            options.use_color);
        return;
    }
    if (snapshot.motion_commands_allowed) {
        if (snapshot.aerial_tracking_available) {
            write_line(output,
                "[2] TRACK AIRBORNE TARGET (HOLD + YAW)",
                Tone::normal,
                options.use_color);
        }
        write_line(output,
            "[R] ABORT MISSION + RTL   |   [Q] QUIT",
            Tone::normal,
            options.use_color);
    } else {
        write_line(output,
            "[Q] QUIT   |   OBSERVATION ONLY",
            Tone::dim,
            options.use_color);
    }
}

} // namespace

std::string render_console(const mission::AppSnapshot& snapshot,
    const std::string_view transport_description,
    const ConsoleViewOptions options,
    const BoardTypeResolver* board_type_resolver) {
    std::ostringstream output;
    write_border(output, {}, options.use_color);
    write_line(output,
        "ONBOARD AUTONOMY   [ " + overall_status(snapshot) + " ]",
        overall_tone(snapshot),
        options.use_color);
    write_line(output,
        std::string("LINK ") +
            (snapshot.vehicle.connected ? "ONLINE" : "WAITING") + "   " +
            std::string(transport_description),
        snapshot.vehicle.connected ? Tone::accent : Tone::dim,
        options.use_color);
    write_vehicle(output,
        snapshot.vehicle,
        board_type_resolver,
        options.use_color);
    write_health(output, snapshot, options.use_color);
    write_camera_and_vision(output, snapshot, options.use_color);
    write_border(output, "MAVLINK / LAST FRAME", options.use_color);
    write_line(output,
        activity_detail("TX", snapshot.tx_activity, snapshot.elapsed),
        Tone::accent,
        options.use_color);
    write_line(output,
        activity_detail("RX", snapshot.rx_activity, snapshot.elapsed),
        Tone::normal,
        options.use_color);
    write_session(output, snapshot, options.use_color);
    write_controls(output, snapshot, options);
    write_border(output, {}, options.use_color);
    return output.str();
}

} // namespace onboard_autonomy::operator_interface::ui
