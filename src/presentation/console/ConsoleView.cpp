#include "onboard_autonomy/presentation/console/ConsoleView.hpp"

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

namespace onboard_autonomy::presentation::console {
namespace {

constexpr std::size_t kConsoleWidth = 88;
constexpr std::size_t kInnerWidth = kConsoleWidth - 2;
constexpr std::size_t kDeviceWidth = 22;
constexpr std::size_t kLinkWidth = 40;
constexpr auto kActivityVisibleFor = std::chrono::milliseconds(550);
constexpr auto kBlinkHalfPeriod = std::chrono::milliseconds(120);
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
    controller,
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
    case Tone::controller:
        return "\x1b[93m";
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

std::string clipped(const std::string_view value, const std::size_t width) {
    if (value.size() <= width) {
        return std::string(value);
    }
    if (width <= 3) {
        return std::string(value.substr(0, width));
    }
    return std::string(value.substr(0, width - 3)) + "...";
}

std::string fitted(const std::string_view value, const std::size_t width) {
    std::string result = clipped(value, width);
    result.append(width - result.size(), ' ');
    return result;
}

std::string centered(const std::string_view value, const std::size_t width) {
    const std::string content = clipped(value, width);
    const auto left = (width - content.size()) / 2;
    const auto right = width - content.size() - left;
    return std::string(left, ' ') + content + std::string(right, ' ');
}

void write_border(std::ostringstream& output,
    const char fill,
    const bool use_color) {
    output << paint("+" + std::string(kInnerWidth, fill) + "+",
                  Tone::chrome,
                  use_color)
           << '\n';
}

void write_line(std::ostringstream& output,
    const std::string_view value,
    const Tone tone,
    const bool use_color) {
    output << paint("|", Tone::chrome, use_color)
           << paint(fitted(value, kInnerWidth), tone, use_color)
           << paint("|", Tone::chrome, use_color) << '\n';
}

void write_centered_line(std::ostringstream& output,
    const std::string_view value,
    const Tone tone,
    const bool use_color) {
    output << paint("|", Tone::chrome, use_color)
           << paint(centered(value, kInnerWidth), tone, use_color)
           << paint("|", Tone::chrome, use_color) << '\n';
}

std::string device_line(const std::string_view value) {
    return "|" + centered(value, kDeviceWidth - 2) + "|";
}

void write_topology_line(std::ostringstream& output,
    const std::string_view companion,
    const Tone companion_tone,
    const std::string_view link,
    const Tone link_tone,
    const std::string_view controller,
    const Tone controller_tone,
    const bool use_color) {
    output << paint("| ", Tone::chrome, use_color)
           << paint(fitted(companion, kDeviceWidth), companion_tone, use_color)
           << paint(fitted(link, kLinkWidth), link_tone, use_color)
           << paint(fitted(controller, kDeviceWidth),
                  controller_tone,
                  use_color)
           << paint(" |", Tone::chrome, use_color) << '\n';
}

void write_header(std::ostringstream& output,
    const std::string_view link_status,
    const std::string_view transport_description,
    const bool connected,
    const bool use_color) {
    constexpr std::string_view title{" ONBOARD AUTONOMY   "};
    constexpr std::string_view link_label{"LINK: "};
    const std::string transport = "   " + std::string(transport_description);
    const auto remaining =
        kInnerWidth - title.size() - link_label.size() - link_status.size();

    output << paint("|", Tone::chrome, use_color)
           << paint(std::string(title), Tone::accent, use_color)
           << paint(std::string(link_label), Tone::normal, use_color)
           << paint(std::string(link_status),
                  connected ? Tone::good : Tone::dim,
                  use_color)
           << paint(fitted(transport, remaining), Tone::dim, use_color)
           << paint("|", Tone::chrome, use_color) << '\n';
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

std::string firmware_detail(const domain::VehicleSnapshot& vehicle) {
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

std::string controller_hardware_name(const domain::VehicleSnapshot& vehicle,
    const BoardTypeResolver* resolver) {
    if (!vehicle.connected || !vehicle.autopilot_metadata.has_value() ||
        vehicle.autopilot_metadata->board_version == 0U) {
        return "FLIGHT CONTROLLER";
    }
    return board_type_name(vehicle.autopilot_metadata->board_version, resolver);
}

std::string board_detail(const domain::VehicleSnapshot& vehicle,
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

std::string telemetry_detail(const application::TelemetryStatus& telemetry) {
    const std::string progress = std::to_string(telemetry.completed_requests) +
                                 "/" + std::to_string(telemetry.total_requests);

    switch (telemetry.state) {
    case application::TelemetrySetupState::waiting_for_vehicle:
        return "TELEMETRY WAITING";
    case application::TelemetrySetupState::configuring:
        return "TELEMETRY SETUP " + progress + " ACCEPTED";
    case application::TelemetrySetupState::active:
        return "TELEMETRY READY / " + std::to_string(telemetry.total_requests) +
               " STREAMS";
    case application::TelemetrySetupState::failed:
        return "TELEMETRY FAILED / " + progress + " ACCEPTED";
    }
    return "TELEMETRY UNKNOWN";
}

std::string companion_link_failsafe_detail(
    const application::CompanionLinkFailsafeSnapshot& failsafe) {
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
    const application::CompanionLinkFailsafeSnapshot& failsafe) {
    switch (failsafe.phase) {
    case application::CompanionLinkFailsafePhase::accepted:
        return Tone::good;
    case application::CompanionLinkFailsafePhase::rejected:
        return Tone::bad;
    case application::CompanionLinkFailsafePhase::reading_parameters:
        return Tone::waiting;
    case application::CompanionLinkFailsafePhase::waiting_for_vehicle:
        return Tone::dim;
    }
    return Tone::bad;
}

Tone metadata_tone(const application::AppSnapshot& snapshot) {
    if (snapshot.telemetry.state == application::TelemetrySetupState::failed) {
        return Tone::bad;
    }
    if (snapshot.telemetry.state == application::TelemetrySetupState::active &&
        snapshot.vehicle.autopilot_metadata.has_value()) {
        return Tone::good;
    }
    return snapshot.vehicle.connected ? Tone::waiting : Tone::dim;
}

std::string camera_phase_name(const application::CameraSnapshot& camera) {
    switch (camera.phase) {
    case application::ports::CameraSourcePhase::starting:
        return "CAMERA STARTING";
    case application::ports::CameraSourcePhase::streaming:
        return "CAMERA STREAMING";
    case application::ports::CameraSourcePhase::reconnecting:
        return "CAMERA RECONNECTING";
    case application::ports::CameraSourcePhase::stopped:
        return "CAMERA STOPPED";
    case application::ports::CameraSourcePhase::failed:
        return "CAMERA FAILED";
    }
    return "CAMERA FAILED";
}

Tone camera_tone(const application::CameraSnapshot& camera) {
    switch (camera.phase) {
    case application::ports::CameraSourcePhase::starting:
        return Tone::waiting;
    case application::ports::CameraSourcePhase::streaming:
        return Tone::good;
    case application::ports::CameraSourcePhase::reconnecting:
        return Tone::waiting;
    case application::ports::CameraSourcePhase::stopped:
        return Tone::dim;
    case application::ports::CameraSourcePhase::failed:
        return Tone::bad;
    }
    return Tone::bad;
}

std::string camera_stream_detail(const application::CameraSnapshot& camera) {
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

std::string camera_latency_detail(const application::CameraSnapshot& camera) {
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

std::string vision_pipeline_detail(const application::VisionSnapshot& vision) {
    std::ostringstream detail;
    detail << "VISION " << vision.detector << "   |   "
           << vision.processed_frames << " FRAMES";
    if (vision.average_processing_ms.has_value()) {
        detail << "   |   " << std::fixed << std::setprecision(1)
               << *vision.average_processing_ms << " MS AVG";
    }
    return detail.str();
}

std::string vision_target_detail(const application::VisionSnapshot& vision) {
    const auto& track = vision.target_track;
    if (track.phase != application::TargetTrackPhase::searching &&
        track.position.has_value()) {
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(2)
               << "TARGET POSITION   |   X RIGHT " << track.position->right_m
               << " M" << "   |   Y DOWN " << track.position->down_m << " M"
               << "   |   Z FORWARD " << track.position->forward_m << " M";
        return detail.str();
    }

    if (vision.latest_targets.empty()) {
        return "TARGET NOT VISIBLE";
    }

    const auto& target = vision.latest_targets.front();
    std::ostringstream detail;
    detail << "TARGET ID " << target.id;
    if (target.pose.has_value()) {
        detail << std::fixed << std::setprecision(2) << "   |   X RIGHT "
               << target.pose->position.right_m << " M" << "   |   Y DOWN "
               << target.pose->position.down_m << " M" << "   |   Z FORWARD "
               << target.pose->position.forward_m << " M";
    } else {
        detail << "   |   CENTER " << std::fixed << std::setprecision(1)
               << target.center.x_px << "/" << target.center.y_px << " PX"
               << "   |   MARGIN " << target.decision_margin
               << "   |   CORRECTED " << target.corrected_bits;
    }
    if (vision.latest_targets.size() > 1U) {
        detail << "   |   " << vision.latest_targets.size() << " TAGS";
    }
    return detail.str();
}

std::optional<std::string> vision_track_status_detail(
    const application::VisionSnapshot& vision) {
    const auto& track = vision.target_track;
    if (track.phase == application::TargetTrackPhase::searching ||
        !track.target_id.has_value()) {
        return std::nullopt;
    }

    std::ostringstream detail;
    detail << "TARGET ID " << *track.target_id;
    if (track.phase == application::TargetTrackPhase::acquiring) {
        detail << "   |   ACQUIRING " << track.consecutive_observations << "/"
               << track.required_observations;
    } else {
        detail << "   |   TRACKING";
    }
    if (track.observation_age_ms.has_value()) {
        detail << std::fixed << std::setprecision(0) << "   |   AGE "
               << *track.observation_age_ms << " MS";
    }
    return detail.str();
}

Tone vision_target_tone(const application::VisionSnapshot& vision) {
    if (vision.target_track.phase == application::TargetTrackPhase::tracking) {
        return Tone::good;
    }
    if (vision.target_track.phase == application::TargetTrackPhase::acquiring) {
        return Tone::waiting;
    }
    return vision.latest_targets.empty() ? Tone::dim : Tone::good;
}

std::string altitude_detail(const domain::VehicleSnapshot& vehicle) {
    if (!vehicle.relative_altitude_m.has_value()) {
        return "--";
    }

    std::ostringstream altitude;
    altitude << std::fixed << std::setprecision(2)
             << *vehicle.relative_altitude_m << " M";
    return altitude.str();
}

std::string gps_detail(const domain::VehicleSnapshot& vehicle) {
    std::string result = gps_fix_name(vehicle.gps_fix_type);
    if (vehicle.satellites_visible.has_value()) {
        result += " / " + std::to_string(*vehicle.satellites_visible) + " SAT";
    }
    return result;
}

std::string battery_detail(const domain::VehicleSnapshot& vehicle) {
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

std::string startup_phase_name(const application::FlightStartupPhase phase) {
    switch (phase) {
    case application::FlightStartupPhase::disabled:
        return "IDLE";
    case application::FlightStartupPhase::waiting_for_vehicle:
    case application::FlightStartupPhase::waiting_for_readiness:
        return "WAITING";
    case application::FlightStartupPhase::setting_guided:
        return "GUIDED";
    case application::FlightStartupPhase::arming:
        return "ARMING";
    case application::FlightStartupPhase::taking_off:
        return "TAKEOFF";
    case application::FlightStartupPhase::completed:
        return "COMPLETE";
    case application::FlightStartupPhase::failed:
        return "FAILED";
    }
    return "UNKNOWN";
}

std::string autonomy_phase_name(const application::AutonomyRuntimePhase phase) {
    switch (phase) {
    case application::AutonomyRuntimePhase::disabled:
        return "IDLE";
    case application::AutonomyRuntimePhase::waiting_for_startup:
        return "WAITING";
    case application::AutonomyRuntimePhase::active:
        return "ACTIVE";
    case application::AutonomyRuntimePhase::landing:
        return "LANDING";
    case application::AutonomyRuntimePhase::completed:
        return "COMPLETE";
    case application::AutonomyRuntimePhase::failed:
        return "FAILED";
    }
    return "UNKNOWN";
}

Tone autonomy_tone(const application::AutonomyRuntimePhase phase) {
    switch (phase) {
    case application::AutonomyRuntimePhase::completed:
        return Tone::good;
    case application::AutonomyRuntimePhase::failed:
        return Tone::bad;
    case application::AutonomyRuntimePhase::active:
    case application::AutonomyRuntimePhase::landing:
        return Tone::accent;
    case application::AutonomyRuntimePhase::disabled:
        return Tone::dim;
    case application::AutonomyRuntimePhase::waiting_for_startup:
        return Tone::waiting;
    }
    return Tone::normal;
}

bool activity_is_fresh(const std::optional<application::LinkActivity>& activity,
    const std::chrono::milliseconds elapsed) {
    return activity.has_value() && elapsed >= activity->observed_at &&
           elapsed - activity->observed_at <= kActivityVisibleFor;
}

bool blink_is_bright(const std::chrono::milliseconds elapsed) {
    return (elapsed.count() / kBlinkHalfPeriod.count()) % 2 == 0;
}

Tone activity_tone(const std::optional<application::LinkActivity>& activity,
    const std::chrono::milliseconds elapsed,
    const Tone group_tone) {
    if (!activity_is_fresh(activity, elapsed) || !blink_is_bright(elapsed)) {
        return Tone::dim;
    }
    return group_tone;
}

std::string activity_label(const application::LinkActivity& activity) {
    return activity.detail.empty()
               ? activity.message_name
               : activity.message_name + ": " + activity.detail;
}

std::string outbound_wire(
    const std::optional<application::LinkActivity>& activity,
    const std::chrono::milliseconds elapsed) {
    if (!activity.has_value() || !activity_is_fresh(activity, elapsed)) {
        return std::string(kLinkWidth - 1, '-') + ">";
    }

    constexpr std::size_t decoration_width = 7;
    const std::string label = clipped(activity_label(activity.value()),
        kLinkWidth - decoration_width);
    const std::string packet = "[ " + label + " ]";
    const char pulse = blink_is_bright(elapsed) ? '=' : '-';
    return std::string(2, pulse) + packet +
           std::string(kLinkWidth - packet.size() - 3, pulse) + ">";
}

std::string inbound_wire(
    const std::optional<application::LinkActivity>& activity,
    const std::chrono::milliseconds elapsed) {
    if (!activity.has_value() || !activity_is_fresh(activity, elapsed)) {
        return "<" + std::string(kLinkWidth - 1, '-');
    }

    constexpr std::size_t decoration_width = 7;
    const std::string label = clipped(activity_label(activity.value()),
        kLinkWidth - decoration_width);
    const std::string packet = "[ " + label + " ]";
    const char pulse = blink_is_bright(elapsed) ? '=' : '-';
    return "<" + std::string(kLinkWidth - packet.size() - 3, pulse) + packet +
           std::string(2, pulse);
}

std::string overall_status(const application::AppSnapshot& snapshot) {
    const auto& vehicle = snapshot.vehicle;
    const auto phase = snapshot.autonomy.phase;
    const bool telemetry_complete = vehicle.gps_fix_type.has_value() &&
                                    vehicle.battery_voltage_v.has_value() &&
                                    vehicle.system_health_known;

    if (phase == application::AutonomyRuntimePhase::completed) {
        return "FLIGHT COMPLETE";
    }
    if (phase == application::AutonomyRuntimePhase::failed ||
        snapshot.flight_startup.phase ==
            application::FlightStartupPhase::failed) {
        return "FLIGHT FAILED";
    }
    if (phase != application::AutonomyRuntimePhase::disabled) {
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

Tone overall_tone(const application::AppSnapshot& snapshot) {
    const std::string status = overall_status(snapshot);
    if (status == "READY" || status == "FLIGHT COMPLETE") {
        return Tone::good;
    }
    if (status == "NOT READY" || status == "FLIGHT FAILED") {
        return Tone::bad;
    }
    if (status == "AUTONOMY RUNNING") {
        return Tone::accent;
    }
    return Tone::waiting;
}

void write_topology(std::ostringstream& output,
    const application::AppSnapshot& snapshot,
    const BoardTypeResolver* resolver,
    const bool use_color) {
    const auto& vehicle = snapshot.vehicle;
    const std::string heartbeat =
        snapshot.companion_heartbeat_active ? "HEARTBEAT ON" : "HEARTBEAT WAIT";
    const std::string controller_mode =
        vehicle.connected ? flight_mode_name(vehicle.flight_mode) +
                                (vehicle.armed ? " / ARMED" : " / DISARMED")
                          : "WAITING FOR LINK";

    write_line(output, "", Tone::normal, use_color);
    write_topology_line(output,
        "+--o--------------o--+",
        Tone::accent,
        centered("MAVLINK", kLinkWidth),
        Tone::dim,
        "/--o--------------o--\\",
        Tone::controller,
        use_color);
    write_topology_line(output,
        device_line("RASPBERRY PI 5"),
        Tone::accent,
        outbound_wire(snapshot.tx_activity, snapshot.elapsed),
        activity_tone(snapshot.tx_activity, snapshot.elapsed, Tone::accent),
        device_line(controller_hardware_name(vehicle, resolver)),
        Tone::controller,
        use_color);
    write_topology_line(output,
        device_line("COMPANION COMPUTER"),
        Tone::accent,
        inbound_wire(snapshot.rx_activity, snapshot.elapsed),
        activity_tone(snapshot.rx_activity, snapshot.elapsed, Tone::controller),
        device_line("FLIGHT CONTROLLER"),
        Tone::controller,
        use_color);
    write_topology_line(output,
        device_line(heartbeat),
        vehicle.connected ? Tone::accent : Tone::dim,
        "",
        Tone::dim,
        device_line(controller_mode),
        vehicle.connected ? Tone::controller : Tone::dim,
        use_color);
    write_topology_line(output,
        device_line("ONBOARD AUTONOMY"),
        Tone::accent,
        "",
        Tone::dim,
        device_line(autopilot_name(vehicle.autopilot_type)),
        Tone::controller,
        use_color);
    write_topology_line(output,
        "+--o--------------o--+",
        Tone::accent,
        "",
        Tone::dim,
        "\\--o--------------o--/",
        Tone::controller,
        use_color);
    write_line(output, "", Tone::normal, use_color);
}

void write_camera_and_vision(std::ostringstream& output,
    const application::AppSnapshot& snapshot,
    const bool use_color) {
    if (snapshot.camera.has_value()) {
        const auto tone = camera_tone(*snapshot.camera);
        write_centered_line(output,
            camera_stream_detail(*snapshot.camera),
            tone,
            use_color);
        write_centered_line(output,
            camera_latency_detail(*snapshot.camera),
            tone,
            use_color);
    }
    if (!snapshot.vision.has_value()) {
        return;
    }

    const auto target_tone = vision_target_tone(*snapshot.vision);
    write_centered_line(output,
        vision_pipeline_detail(*snapshot.vision),
        Tone::accent,
        use_color);
    if (const auto status = vision_track_status_detail(*snapshot.vision);
        status.has_value()) {
        write_centered_line(output, *status, target_tone, use_color);
    }
    write_centered_line(output,
        vision_target_detail(*snapshot.vision),
        target_tone,
        use_color);
}

void write_vehicle_status(std::ostringstream& output,
    const application::AppSnapshot& snapshot,
    const BoardTypeResolver* resolver,
    const bool use_color) {
    const auto& vehicle = snapshot.vehicle;
    write_centered_line(output,
        "[ " + overall_status(snapshot) + " ]",
        overall_tone(snapshot),
        use_color);
    write_centered_line(output,
        telemetry_detail(snapshot.telemetry) + "   |   " +
            firmware_detail(vehicle),
        metadata_tone(snapshot),
        use_color);
    write_centered_line(output,
        board_detail(vehicle, resolver),
        metadata_tone(snapshot),
        use_color);
    write_centered_line(output,
        companion_link_failsafe_detail(snapshot.companion_link_failsafe),
        companion_link_failsafe_tone(snapshot.companion_link_failsafe),
        use_color);
    write_camera_and_vision(output, snapshot, use_color);
    write_centered_line(output,
        "MODE " + flight_mode_name(vehicle.flight_mode) + "   |   ALT " +
            altitude_detail(vehicle) + "   |   GPS " + gps_detail(vehicle) +
            "   |   BAT " + battery_detail(vehicle),
        vehicle.connected ? Tone::normal : Tone::dim,
        use_color);
    const bool healthy = vehicle.warnings.empty();
    const std::string warning =
        healthy ? (vehicle.connected ? "NO ACTIVE WARNINGS"
                                     : "WARNINGS AVAILABLE AFTER CONNECTION")
                : "! " + vehicle.warnings.front();
    const Tone warning_tone =
        healthy ? (vehicle.connected ? Tone::good : Tone::dim) : Tone::bad;
    write_centered_line(output, warning, warning_tone, use_color);
}

void write_runtime_footer(std::ostringstream& output,
    const application::AppSnapshot& snapshot,
    const bool use_color) {
    const auto& startup = snapshot.flight_startup;
    const auto& autonomy = snapshot.autonomy;
    const bool startup_finished =
        startup.phase == application::FlightStartupPhase::completed ||
        startup.phase == application::FlightStartupPhase::disabled;

    write_border(output, '-', use_color);
    write_line(output,
        " AUTONOMY: " + autonomy_phase_name(autonomy.phase) +
            " | STARTUP: " + startup_phase_name(startup.phase),
        autonomy_tone(autonomy.phase),
        use_color);
    write_line(output,
        " " + (startup_finished ? autonomy.detail : startup.detail),
        autonomy_tone(autonomy.phase),
        use_color);
    write_centered_line(output,
        snapshot.motion_commands_allowed
            ? "[S] START AGAIN     [Q] QUIT"
            : "LIVE VIEW     MOTION KEYS DISABLED     CTRL+C EXIT",
        snapshot.motion_commands_allowed ? Tone::normal : Tone::dim,
        use_color);
    write_border(output, '=', use_color);
}

} // namespace

std::string render_console(const application::AppSnapshot& snapshot,
    const std::string_view transport_description,
    const bool use_color,
    const BoardTypeResolver* board_type_resolver) {
    std::ostringstream output;
    write_border(output, '=', use_color);
    write_header(output,
        snapshot.vehicle.connected ? "ONLINE" : "WAITING",
        transport_description,
        snapshot.vehicle.connected,
        use_color);
    write_border(output, '-', use_color);
    write_topology(output, snapshot, board_type_resolver, use_color);
    write_vehicle_status(output, snapshot, board_type_resolver, use_color);
    write_runtime_footer(output, snapshot, use_color);
    return output.str();
}

} // namespace onboard_autonomy::presentation::console
