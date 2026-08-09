#include "TestCases.hpp"

#include "onboard_autonomy/presentation/cli/CommandLine.hpp"

#include <initializer_list>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using onboard_autonomy::presentation::cli::CameraBackend;
using onboard_autonomy::presentation::cli::CommandLineOptions;
using onboard_autonomy::presentation::cli::TransportBackend;
using onboard_autonomy::presentation::cli::command_line_help;
using onboard_autonomy::presentation::cli::parse_command_line;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

CommandLineOptions parse(
    const std::initializer_list<std::string_view> arguments
) {
    return parse_command_line({arguments.begin(), arguments.end()});
}

void require_rejected(
    const std::initializer_list<std::string_view> arguments,
    const std::string_view expected_message
) {
    try {
        static_cast<void>(parse(arguments));
    } catch (const std::invalid_argument& error) {
        require(
            std::string_view{error.what()}.find(expected_message) !=
                std::string_view::npos,
            "CLI rejection must explain the invalid combination"
        );
        return;
    }
    throw std::runtime_error("invalid CLI combination was accepted");
}

void defaults_are_documented_udp_observation_mode() {
    const auto options = parse({});
    require(
        options.transport == TransportBackend::udp &&
            options.udp_bind == "0.0.0.0" &&
            options.udp_port == 14550 &&
            options.snapshot_interval_ms == 1000 &&
            !options.sitl_mode && !options.autonomous,
        "empty CLI must preserve documented observation-only defaults"
    );
}

void transport_groups_are_explicit_and_exclusive() {
    const auto serial = parse(
        {
            "--transport",
            "serial",
            "--serial-device",
            "/dev/ttyACM0",
            "--baud",
            "57600",
        }
    );
    require(
        serial.transport == TransportBackend::serial &&
            serial.serial_device == "/dev/ttyACM0" &&
            serial.baud_rate == 57600,
        "serial group must map to typed serial options"
    );

    require_rejected(
        {"--transport", "serial", "--udp-port", "14551"},
        "--udp-bind and --udp-port require --transport udp"
    );
    require_rejected(
        {"--transport", "udp", "--serial-device", "/dev/ttyACM0"},
        "require --transport serial"
    );
    require_rejected(
        {"--transport", "serial"},
        "requires --serial-device"
    );
}

void camera_and_autonomy_dependencies_are_validated() {
    const auto gazebo = parse(
        {
            "--transport", "udp",
            "--sitl",
            "--camera",
            "--camera-source", "gstreamer",
            "--camera-udp-port", "5601",
            "--camera-width", "640",
            "--camera-height", "480",
            "--apriltag",
            "--camera-calibration", "camera.json",
            "--apriltag-size-mm", "2000",
            "--camera-extrinsics", "mount.json",
            "--autonomous",
            "--exit-after-autonomy",
        }
    );
    require(
        gazebo.camera_enabled &&
            gazebo.camera_backend == CameraBackend::gstreamer &&
            gazebo.autonomous && gazebo.exit_after_autonomy,
        "valid Gazebo autonomy group must parse"
    );

    require_rejected(
        {"--camera-source", "gstreamer"},
        "require --camera"
    );
    require_rejected(
        {"--camera", "--camera-source", "gstreamer", "--camera-fps", "30"},
        "available only"
    );
    require_rejected(
        {"--camera", "--autonomous"},
        "requires --camera-extrinsics"
    );
    require_rejected(
        {"--camera", "--camera-preview-port", "8081"},
        "requires --camera-preview"
    );
    require_rejected(
        {"--interactive", "--json"},
        "cannot be combined"
    );
}

void simulated_wind_is_typed_and_sitl_only() {
    const auto options = parse(
        {"--sitl", "--sim-wind", "3.0", "270", "0.6"}
    );
    require(
        options.simulated_wind.has_value() &&
            options.simulated_wind->speed_m_s == 3.0 &&
            options.simulated_wind->direction_from_deg == 270.0 &&
            options.simulated_wind->turbulence_m_s == 0.6,
        "simulation wind must map to a typed profile"
    );

    require_rejected(
        {"--sim-wind", "3", "270", "0.6"},
        "requires --sitl"
    );
    require_rejected(
        {"--sitl", "--sim-wind", "-1", "270", "0.6"},
        "speed must be finite and non-negative"
    );
    require_rejected(
        {"--sitl", "--sim-wind", "3", "360", "0.6"},
        "direction must be in [0, 360)"
    );
}

void removed_options_provide_migration_guidance() {
    require_rejected(
        {"--serial", "/dev/ttyACM0"},
        "--transport serial --serial-device DEVICE"
    );
    require_rejected(
        {"--scenario", "5"},
        "was removed; use --autonomous"
    );
}

void help_is_grouped_by_operator_concern() {
    const auto help = command_line_help();
    for (const std::string_view heading : {
             "Transport (default: udp):",
             "Camera source:",
             "Vision and preview:",
             "Autonomy and safety:",
             "Output and interaction:",
         }) {
        require(
            help.find(heading) != std::string::npos,
            "CLI help is missing a required category"
        );
    }
}

}  // namespace

void run_command_line_tests() {
    defaults_are_documented_udp_observation_mode();
    transport_groups_are_explicit_and_exclusive();
    camera_and_autonomy_dependencies_are_validated();
    simulated_wind_is_typed_and_sitl_only();
    removed_options_provide_migration_guidance();
    help_is_grouped_by_operator_concern();
}
