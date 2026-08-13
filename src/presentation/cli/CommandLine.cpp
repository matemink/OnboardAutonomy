#include "onboard_autonomy/presentation/cli/CommandLine.hpp"

#include <charconv>
#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>

namespace onboard_autonomy::presentation::cli {
namespace {

struct ExplicitOptions {
    bool udp_bind{false};
    bool udp_port{false};
    bool serial_device{false};
    bool baud_rate{false};
    bool camera_source{false};
    bool camera_udp_port{false};
    bool camera_width{false};
    bool camera_height{false};
    bool camera_fps{false};
    bool camera_preview_port{false};
};

template <typename T>
T parse_number(const std::string_view text, const std::string_view option) {
    T value{};
    const auto [end, error] =
        std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size()) {
        throw std::invalid_argument(
            "Invalid numeric value for " + std::string(option));
    }
    return value;
}

void validate_transport(const CommandLineOptions& options,
    const ExplicitOptions& explicit_options) {
    if (options.transport == TransportBackend::udp) {
        if (explicit_options.serial_device || explicit_options.baud_rate) {
            throw std::invalid_argument("--serial-device and --baud require "
                                        "--transport serial");
        }
        if (options.udp_bind.empty()) {
            throw std::invalid_argument("--udp-bind cannot be empty");
        }
        if (options.udp_port == 0) {
            throw std::invalid_argument("--udp-port must be positive");
        }
        return;
    }

    if (explicit_options.udp_bind || explicit_options.udp_port) {
        throw std::invalid_argument(
            "--udp-bind and --udp-port require --transport udp");
    }
    if (options.serial_device.empty()) {
        throw std::invalid_argument(
            "--transport serial requires --serial-device DEVICE");
    }
    if (options.baud_rate == 0) {
        throw std::invalid_argument("--baud must be positive");
    }
    if (options.sitl_mode) {
        throw std::invalid_argument(
            "--sitl cannot be combined with --transport serial");
    }
}

void validate_camera(const CommandLineOptions& options,
    const ExplicitOptions& explicit_options) {
    const bool camera_setting_used =
        explicit_options.camera_source || explicit_options.camera_udp_port ||
        explicit_options.camera_width || explicit_options.camera_height ||
        explicit_options.camera_fps || explicit_options.camera_preview_port ||
        options.apriltag_enabled || options.camera_preview_enabled ||
        !options.camera_calibration_file.empty() ||
        !options.camera_extrinsics_file.empty() ||
        options.apriltag_tag_size_m.has_value();
    if (camera_setting_used && !options.camera_enabled) {
        throw std::invalid_argument(
            "camera, vision, and preview options require --camera");
    }
    if (!options.camera_enabled) {
        return;
    }
    if (options.camera_width == 0 || options.camera_height == 0 ||
        options.camera_width % 2 != 0 || options.camera_height % 2 != 0) {
        throw std::invalid_argument(
            "camera width and height must be positive even values");
    }
    if (options.camera_backend == CameraBackend::rpicam &&
        explicit_options.camera_udp_port) {
        throw std::invalid_argument(
            "--camera-udp-port requires --camera-source gstreamer");
    }
    if (options.camera_backend == CameraBackend::gstreamer &&
        explicit_options.camera_fps) {
        throw std::invalid_argument(
            "--camera-fps is available only for --camera-source rpicam");
    }
    if (options.camera_backend == CameraBackend::rpicam &&
        options.camera_fps == 0) {
        throw std::invalid_argument("--camera-fps must be positive");
    }
    if (options.camera_backend == CameraBackend::gstreamer &&
        options.camera_udp_port == 0) {
        throw std::invalid_argument("--camera-udp-port must be positive");
    }
    if (explicit_options.camera_preview_port &&
        !options.camera_preview_enabled) {
        throw std::invalid_argument(
            "--camera-preview-port requires --camera-preview");
    }
    if (options.camera_preview_enabled && options.camera_preview_port == 0) {
        throw std::invalid_argument("--camera-preview-port must be positive");
    }
}

void validate_vision(const CommandLineOptions& options) {
    if (!options.camera_enabled) {
        return;
    }
    const bool has_calibration = !options.camera_calibration_file.empty();
    if (has_calibration != options.apriltag_tag_size_m.has_value()) {
        throw std::invalid_argument(
            "--camera-calibration and --apriltag-size-mm must be "
            "provided together");
    }
    if (has_calibration && !options.apriltag_enabled) {
        throw std::invalid_argument(
            "calibrated AprilTag pose requires --apriltag");
    }
    if (options.apriltag_tag_size_m.has_value() &&
        (!std::isfinite(*options.apriltag_tag_size_m) ||
            *options.apriltag_tag_size_m <= 0.0)) {
        throw std::invalid_argument(
            "--apriltag-size-mm must be finite and positive");
    }
    const bool has_extrinsics = !options.camera_extrinsics_file.empty();
    if (has_extrinsics && !has_calibration) {
        throw std::invalid_argument(
            "--camera-extrinsics requires calibrated AprilTag pose");
    }
    if (options.autonomous && !has_extrinsics) {
        throw std::invalid_argument(
            "--autonomous requires --camera-extrinsics");
    }
}

void validate_options(const CommandLineOptions& options,
    const ExplicitOptions& explicit_options) {
    if (options.show_help) {
        return;
    }
    if (options.snapshot_interval_ms == 0) {
        throw std::invalid_argument("--snapshot-ms must be positive");
    }
    if (options.exit_after_autonomy && !options.autonomous) {
        throw std::invalid_argument(
            "--exit-after-autonomy requires --autonomous");
    }
    if (options.interactive && options.json_output) {
        throw std::invalid_argument(
            "--interactive cannot be combined with --json");
    }
    if (options.simulated_wind.has_value()) {
        const auto& wind = *options.simulated_wind;
        if (!options.sitl_mode) {
            throw std::invalid_argument("--sim-wind requires --sitl");
        }
        if (!std::isfinite(wind.speed_m_s) || wind.speed_m_s < 0.0) {
            throw std::invalid_argument(
                "--sim-wind speed must be finite and non-negative");
        }
        if (!std::isfinite(wind.direction_from_deg) ||
            wind.direction_from_deg < 0.0 || wind.direction_from_deg >= 360.0) {
            throw std::invalid_argument(
                "--sim-wind direction must be in [0, 360)");
        }
        if (!std::isfinite(wind.turbulence_m_s) || wind.turbulence_m_s < 0.0) {
            throw std::invalid_argument(
                "--sim-wind turbulence must be finite and non-negative");
        }
    }
    validate_transport(options, explicit_options);
    validate_camera(options, explicit_options);
    validate_vision(options);
}

class ArgumentParser {
  public:
    explicit ArgumentParser(const std::vector<std::string_view>& arguments)
        : arguments_(arguments) {}

    CommandLineOptions parse() {
        while (index_ < arguments_.size()) {
            const auto argument = arguments_[index_++];
            if (!parse_transport(argument) && !parse_camera(argument) &&
                !parse_runtime(argument)) {
                reject(argument);
            }
        }
        validate_options(options_, explicit_);
        return options_;
    }

  private:
    std::string_view value_after(const std::string_view option) {
        if (index_ >= arguments_.size()) {
            throw std::invalid_argument(
                "Missing value after " + std::string(option));
        }
        return arguments_[index_++];
    }

    bool parse_transport(const std::string_view argument) {
        if (argument == "--transport") {
            const auto value = value_after(argument);
            if (value == "udp") {
                options_.transport = TransportBackend::udp;
            } else if (value == "serial") {
                options_.transport = TransportBackend::serial;
            } else {
                throw std::invalid_argument(
                    "--transport must be udp or serial");
            }
        } else if (argument == "--udp-bind") {
            options_.udp_bind = value_after(argument);
            explicit_.udp_bind = true;
        } else if (argument == "--udp-port") {
            options_.udp_port =
                parse_number<std::uint16_t>(value_after(argument), argument);
            explicit_.udp_port = true;
        } else if (argument == "--serial-device") {
            options_.serial_device = value_after(argument);
            explicit_.serial_device = true;
        } else if (argument == "--baud") {
            options_.baud_rate =
                parse_number<std::uint32_t>(value_after(argument), argument);
            explicit_.baud_rate = true;
        } else {
            return false;
        }
        return true;
    }

    bool parse_camera(const std::string_view argument) {
        if (argument == "--camera") {
            options_.camera_enabled = true;
        } else if (argument == "--camera-source") {
            const auto source = value_after(argument);
            if (source != "rpicam" && source != "gstreamer") {
                throw std::invalid_argument(
                    "--camera-source must be rpicam or gstreamer");
            }
            options_.camera_backend = source == "rpicam"
                                          ? CameraBackend::rpicam
                                          : CameraBackend::gstreamer;
            explicit_.camera_source = true;
        } else if (argument == "--camera-udp-port") {
            options_.camera_udp_port =
                parse_number<std::uint16_t>(value_after(argument), argument);
            explicit_.camera_udp_port = true;
        } else if (argument == "--camera-width") {
            options_.camera_width =
                parse_number<std::uint32_t>(value_after(argument), argument);
            explicit_.camera_width = true;
        } else if (argument == "--camera-height") {
            options_.camera_height =
                parse_number<std::uint32_t>(value_after(argument), argument);
            explicit_.camera_height = true;
        } else if (argument == "--camera-fps") {
            options_.camera_fps =
                parse_number<std::uint32_t>(value_after(argument), argument);
            explicit_.camera_fps = true;
        } else if (argument == "--apriltag") {
            options_.apriltag_enabled = true;
        } else if (argument == "--camera-calibration") {
            options_.camera_calibration_file = value_after(argument);
        } else if (argument == "--camera-extrinsics") {
            options_.camera_extrinsics_file = value_after(argument);
        } else if (argument == "--apriltag-size-mm") {
            options_.apriltag_tag_size_m =
                parse_number<double>(value_after(argument), argument) / 1000.0;
        } else if (argument == "--camera-preview") {
            options_.camera_preview_enabled = true;
        } else if (argument == "--camera-preview-port") {
            options_.camera_preview_port =
                parse_number<std::uint16_t>(value_after(argument), argument);
            explicit_.camera_preview_port = true;
        } else {
            return false;
        }
        return true;
    }

    bool parse_runtime(const std::string_view argument) {
        if (argument == "--snapshot-ms") {
            options_.snapshot_interval_ms =
                parse_number<std::uint32_t>(value_after(argument), argument);
        } else if (argument == "--board-types") {
            options_.board_types_file = value_after(argument);
        } else if (argument == "--json") {
            options_.json_output = true;
        } else if (argument == "--sitl") {
            options_.sitl_mode = true;
        } else if (argument == "--sim-wind") {
            options_.simulated_wind = application::SimulatedWindProfile{
                .speed_m_s =
                    parse_number<double>(value_after(argument), argument),
                .direction_from_deg =
                    parse_number<double>(value_after(argument), argument),
                .turbulence_m_s =
                    parse_number<double>(value_after(argument), argument),
            };
        } else if (argument == "--autonomous") {
            options_.autonomous = true;
        } else if (argument == "--exit-after-autonomy") {
            options_.exit_after_autonomy = true;
        } else if (argument == "--interactive") {
            options_.interactive = true;
        } else if (argument == "--help" || argument == "-h") {
            options_.show_help = true;
        } else {
            return false;
        }
        return true;
    }

    [[noreturn]] static void reject(const std::string_view argument) {
        if (argument == "--serial") {
            throw std::invalid_argument("--serial was renamed; use --transport "
                                        "serial --serial-device DEVICE");
        }
        if (argument == "--scenario" || argument == "--demo-flight" ||
            argument == "--exit-after-scenario") {
            throw std::invalid_argument(
                std::string(argument) + " was removed; use --autonomous");
        }
        throw std::invalid_argument(
            "Unknown argument: " + std::string(argument));
    }

    const std::vector<std::string_view>& arguments_;
    std::size_t index_{0};
    CommandLineOptions options_;
    ExplicitOptions explicit_;
};

} // namespace

CommandLineOptions parse_command_line(
    const std::vector<std::string_view>& arguments) {
    return ArgumentParser{arguments}.parse();
}

std::string command_line_help() {
    return "OnboardAutonomy companion service\n\n"
           "Usage:\n"
           "  onboard_autonomy [options]\n\n"
           "Transport (default: udp):\n"
           "  --transport udp|serial       MAVLink transport\n"
           "  --udp-bind ADDRESS           UDP bind address, default 0.0.0.0\n"
           "  --udp-port N                 UDP port, default 14550\n"
           "  --serial-device DEVICE       Linux serial device\n"
           "  --baud N                     Serial baud, default 115200\n\n"
           "Camera source:\n"
           "  --camera                     Enable camera capture\n"
           "  --camera-source SOURCE       rpicam (default) or gstreamer\n"
           "  --camera-width N             Frame width, default 640\n"
           "  --camera-height N            Frame height, default 480\n"
           "  --camera-fps N               rpicam FPS, default 30\n"
           "  --camera-udp-port N          GStreamer RTP port, default 5601\n\n"
           "Vision and preview:\n"
           "  --apriltag                   Detect tagStandard41h12 targets\n"
           "  --camera-calibration FILE    Verified calibration JSON\n"
           "  --camera-extrinsics FILE     Camera-to-body-FRD JSON\n"
           "  --apriltag-size-mm N         Detection-corner span\n"
           "  --camera-preview             Serve grayscale HTTP preview\n"
           "  --camera-preview-port N      HTTP port, default 8080\n\n"
           "Autonomy and safety:\n"
           "  --sitl                       Assert UDP peer is SITL\n"
           "  --sim-wind MPS DEG GUST      Show configured SITL wind profile\n"
           "  --autonomous                 Run startup and vision autonomy\n"
           "  --exit-after-autonomy        Exit after completion or failure\n\n"
           "Output and interaction:\n"
           "  --interactive                Enable LAND and quit keys\n"
           "  --json                       Print JSON snapshots\n"
           "  --snapshot-ms N              Output interval, default 1000\n"
           "  --board-types FILE           Override ArduPilot board table\n"
           "  --help, -h                   Show this help\n";
}

} // namespace onboard_autonomy::presentation::cli
