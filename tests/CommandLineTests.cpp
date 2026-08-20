#include "TestCases.hpp"

#include "onboard_autonomy/presentation/cli/CommandLine.hpp"

#include <initializer_list>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace {

using onboard_autonomy::presentation::cli::CommandLineOptions;
using onboard_autonomy::presentation::cli::GStreamerCameraOptions;
using onboard_autonomy::presentation::cli::HardwareLaunchOptions;
using onboard_autonomy::presentation::cli::parse_command_line;
using onboard_autonomy::presentation::cli::SerialConnectionOptions;
using onboard_autonomy::presentation::cli::SimulationLaunchOptions;
using onboard_autonomy::presentation::cli::UdpConnectionOptions;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

CommandLineOptions parse(
    const std::initializer_list<std::string_view> arguments) {
    return parse_command_line({arguments.begin(), arguments.end()});
}

void require_rejected(const std::initializer_list<std::string_view> arguments,
    const std::string_view expected_message) {
    try {
        static_cast<void>(parse(arguments));
    } catch (const std::invalid_argument& error) {
        require(std::string_view{error.what()}.find(expected_message) !=
                    std::string_view::npos,
            "CLI rejection must explain the invalid combination");
        return;
    }
    throw std::runtime_error("invalid CLI combination was accepted");
}

void defaults_are_documented_udp_observation_mode() {
    const auto options = parse({});
    const auto* hardware = std::get_if<HardwareLaunchOptions>(&options);
    const auto* udp = hardware == nullptr ? nullptr
                                          : std::get_if<UdpConnectionOptions>(
                                                &hardware->connection);
    require(udp != nullptr && udp->bind_address == "0.0.0.0" &&
                udp->port == 14550 &&
                hardware->operator_interface.snapshot_interval_ms == 1000 &&
                !hardware->autonomy.enabled,
        "empty CLI must preserve documented observation-only defaults");
}

void transport_groups_are_explicit_and_exclusive() {
    const auto serial = parse({
        "--transport",
        "serial",
        "--serial-device",
        "/dev/ttyACM0",
        "--baud",
        "57600",
    });
    const auto* hardware = std::get_if<HardwareLaunchOptions>(&serial);
    const auto* serial_connection =
        hardware == nullptr
            ? nullptr
            : std::get_if<SerialConnectionOptions>(&hardware->connection);
    require(serial_connection != nullptr &&
                serial_connection->device == "/dev/ttyACM0" &&
                serial_connection->baud_rate == 57600,
        "serial group must map to typed serial options");

    require_rejected({"--transport", "serial", "--udp-port", "14551"},
        "--udp-bind and --udp-port require --transport udp");
    require_rejected({"--transport", "udp", "--serial-device", "/dev/ttyACM0"},
        "require --transport serial");
    require_rejected({"--transport", "serial"}, "requires --serial-device");
    require_rejected(
        {"--sitl", "--transport", "serial", "--serial-device", "/dev/ttyACM0"},
        "--sitl cannot be combined with --transport serial");
}

void camera_and_autonomy_dependencies_are_validated() {
    const auto gazebo = parse({
        "--transport",
        "udp",
        "--sitl",
        "--camera",
        "--camera-source",
        "gstreamer",
        "--camera-udp-port",
        "5601",
        "--camera-width",
        "640",
        "--camera-height",
        "480",
        "--apriltag",
        "--camera-calibration",
        "camera.json",
        "--apriltag-size-mm",
        "2000",
        "--camera-extrinsics",
        "mount.json",
        "--autonomous",
        "--exit-after-autonomy",
    });
    const auto* simulation = std::get_if<SimulationLaunchOptions>(&gazebo);
    require(simulation != nullptr && simulation->camera.has_value() &&
                std::holds_alternative<GStreamerCameraOptions>(
                    simulation->camera->source) &&
                simulation->autonomy.enabled &&
                simulation->autonomy.exit_when_finished,
        "valid Gazebo autonomy group must parse");

    require_rejected({"--camera-source", "gstreamer"}, "require --camera");
    require_rejected(
        {"--camera", "--camera-source", "gstreamer", "--camera-fps", "30"},
        "available only");
    require_rejected({"--camera", "--autonomous"},
        "requires --camera-extrinsics");
    require_rejected({"--camera", "--camera-preview-port", "8081"},
        "requires --camera-preview");
    require_rejected({"--interactive", "--json"}, "cannot be combined");
}

void simulated_wind_is_typed_and_sitl_only() {
    const auto options = parse({"--sitl", "--sim-wind", "3.0", "270", "0.6"});
    const auto* simulation = std::get_if<SimulationLaunchOptions>(&options);
    require(simulation != nullptr && simulation->wind.has_value() &&
                simulation->wind->speed_m_s == 3.0 &&
                simulation->wind->direction_from_deg == 270.0 &&
                simulation->wind->turbulence_m_s == 0.6,
        "simulation wind must map to a typed profile");

    require_rejected({"--sim-wind", "3", "270", "0.6"}, "requires --sitl");
    require_rejected({"--sitl", "--sim-wind", "-1", "270", "0.6"},
        "speed must be finite and non-negative");
    require_rejected({"--sitl", "--sim-wind", "3", "360", "0.6"},
        "direction must be in [0, 360)");
}

void removed_options_provide_migration_guidance() {
    require_rejected({"--serial", "/dev/ttyACM0"},
        "--transport serial --serial-device DEVICE");
    require_rejected({"--scenario", "5"}, "was removed; use --autonomous");
}

} // namespace

void run_command_line_tests() {
    defaults_are_documented_udp_observation_mode();
    transport_groups_are_explicit_and_exclusive();
    camera_and_autonomy_dependencies_are_validated();
    simulated_wind_is_typed_and_sitl_only();
    removed_options_provide_migration_guidance();
}
