#include "onboard_autonomy/operator/cli/CommandLine.hpp"

#include <charconv>
#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace onboard_autonomy::operator_interface::cli {
namespace {

enum class TransportBackend {
    udp,
    serial,
};

enum class CameraBackend {
    rpicam,
    gstreamer,
};

// Temporary parser buffer. Only validated launch states leave this file.
struct LaunchArgumentsDraft {
    std::optional<double> apriltag_tag_size_m;
    std::string udp_bind{defaults::kUdpBindAddress};
    std::string serial_device;
    std::string camera_calibration_file;
    std::string camera_extrinsics_file;
    std::string board_types_file;
    std::string diagnostic_log_file;
    std::string forward_detector_model_file;
    std::optional<mission::SimulatedWindProfile> simulated_wind;
    std::optional<std::uint16_t> forward_camera_udp_port;
    TransportBackend transport{TransportBackend::udp};
    std::uint32_t baud_rate{defaults::kSerialBaudRate};
    std::uint32_t snapshot_interval_ms{defaults::kSnapshotIntervalMs};
    CameraBackend camera_backend{CameraBackend::rpicam};
    std::uint32_t camera_width{defaults::kCameraFrameWidth};
    std::uint32_t camera_height{defaults::kCameraFrameHeight};
    std::uint32_t camera_fps{defaults::kCameraFramesPerSecond};
    std::uint16_t udp_port{defaults::kMavlinkUdpPort};
    std::uint16_t camera_udp_port{defaults::kCameraUdpPort};
    std::uint16_t camera_preview_port{defaults::kCameraPreviewPort};
    bool camera_enabled{};
    bool apriltag_enabled{};
    bool camera_preview_enabled{};
    bool json_output{};
    bool sitl_mode{};
    bool autonomous{};
    bool exit_after_autonomy{};
    bool interactive{};
};

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
    bool diagnostic_log_file{false};
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

void validate_transport(const LaunchArgumentsDraft& options,
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

void validate_camera(const LaunchArgumentsDraft& options,
    const ExplicitOptions& explicit_options) {
    const bool camera_setting_used =
        explicit_options.camera_source || explicit_options.camera_udp_port ||
        explicit_options.camera_width || explicit_options.camera_height ||
        explicit_options.camera_fps || explicit_options.camera_preview_port ||
        options.forward_camera_udp_port.has_value() ||
        !options.forward_detector_model_file.empty() ||
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
    if (options.forward_camera_udp_port.has_value()) {
        if (!options.camera_preview_enabled) {
            throw std::invalid_argument(
                "--forward-camera-udp-port requires --camera-preview");
        }
        if (*options.forward_camera_udp_port == 0U) {
            throw std::invalid_argument(
                "--forward-camera-udp-port must be positive");
        }
        if (options.camera_backend == CameraBackend::gstreamer &&
            *options.forward_camera_udp_port == options.camera_udp_port) {
            throw std::invalid_argument(
                "forward and downward camera UDP ports must differ");
        }
    }
    if (!options.forward_detector_model_file.empty() &&
        !options.forward_camera_udp_port.has_value()) {
        throw std::invalid_argument(
            "--forward-detector-model requires --forward-camera-udp-port");
    }
}

void validate_vision(const LaunchArgumentsDraft& options) {
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

void validate_options(const LaunchArgumentsDraft& options,
    const ExplicitOptions& explicit_options) {
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
    if (options.diagnostic_log_file.empty() &&
        explicit_options.diagnostic_log_file) {
        throw std::invalid_argument("--diagnostic-log cannot be empty");
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
            wind.direction_from_deg < 0.0 ||
            wind.direction_from_deg >= defaults::kDegreesPerCircle) {
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

MavlinkConnectionOptions make_connection_options(
    const LaunchArgumentsDraft& draft) {
    if (draft.transport == TransportBackend::udp) {
        return UdpConnectionOptions{
            .bind_address = draft.udp_bind,
            .port = draft.udp_port,
        };
    }
    return SerialConnectionOptions{
        .device = draft.serial_device,
        .baud_rate = draft.baud_rate,
    };
}

CameraSourceOptions make_camera_source_options(
    const LaunchArgumentsDraft& draft) {
    if (draft.camera_backend == CameraBackend::rpicam) {
        return RpicamOptions{
            .frames_per_second = draft.camera_fps,
        };
    }
    return GStreamerCameraOptions{
        .udp_port = draft.camera_udp_port,
    };
}

std::optional<CameraOptions> make_camera_options(
    const LaunchArgumentsDraft& draft) {
    if (!draft.camera_enabled) {
        return std::nullopt;
    }

    std::optional<AprilTagOptions> apriltag;
    if (draft.apriltag_enabled) {
        apriltag = AprilTagOptions{
            .calibration_file = draft.camera_calibration_file,
            .extrinsics_file = draft.camera_extrinsics_file,
            .tag_size_m = draft.apriltag_tag_size_m,
        };
    }

    return CameraOptions{
        .source = make_camera_source_options(draft),
        .frame_width = draft.camera_width,
        .frame_height = draft.camera_height,
        .apriltag = std::move(apriltag),
    };
}

CommandLineOptions make_command_line_options(
    const LaunchArgumentsDraft& draft) {
    auto camera = make_camera_options(draft);
    const AutonomyOptions autonomy{
        .enabled = draft.autonomous,
        .exit_when_finished = draft.exit_after_autonomy,
    };
    const OperatorInterfaceOptions operator_interface{
        .interactive = draft.interactive,
        .json_output = draft.json_output,
        .snapshot_interval_ms = draft.snapshot_interval_ms,
        .board_types_file = draft.board_types_file,
    };
    const DiagnosticsOptions diagnostics{
        .camera_preview = draft.camera_preview_enabled
                              ? std::optional{CameraPreviewOptions{
                                    .port = draft.camera_preview_port,
                                }}
                              : std::nullopt,
        .forward_camera =
            draft.forward_camera_udp_port.has_value()
                ? std::optional{ForwardCameraOptions{
                      .udp_port = *draft.forward_camera_udp_port,
                      .detector_model_file = draft.forward_detector_model_file,
                  }}
                : std::nullopt,
        .log_file = draft.diagnostic_log_file,
    };

    if (draft.sitl_mode) {
        return SimulationLaunchOptions{
            .connection =
                {
                    .bind_address = draft.udp_bind,
                    .port = draft.udp_port,
                },
            .camera = std::move(camera),
            .wind = draft.simulated_wind,
            .autonomy = autonomy,
            .operator_interface = operator_interface,
            .diagnostics = diagnostics,
        };
    }

    return HardwareLaunchOptions{
        .connection = make_connection_options(draft),
        .camera = std::move(camera),
        .autonomy = autonomy,
        .operator_interface = operator_interface,
        .diagnostics = diagnostics,
    };
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
        validate_options(draft_, explicit_);
        return make_command_line_options(draft_);
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
                draft_.transport = TransportBackend::udp;
            } else if (value == "serial") {
                draft_.transport = TransportBackend::serial;
            } else {
                throw std::invalid_argument(
                    "--transport must be udp or serial");
            }
        } else if (argument == "--udp-bind") {
            draft_.udp_bind = value_after(argument);
            explicit_.udp_bind = true;
        } else if (argument == "--udp-port") {
            draft_.udp_port =
                parse_number<std::uint16_t>(value_after(argument), argument);
            explicit_.udp_port = true;
        } else if (argument == "--serial-device") {
            draft_.serial_device = value_after(argument);
            explicit_.serial_device = true;
        } else if (argument == "--baud") {
            draft_.baud_rate =
                parse_number<std::uint32_t>(value_after(argument), argument);
            explicit_.baud_rate = true;
        } else {
            return false;
        }
        return true;
    }

    bool parse_camera(const std::string_view argument) {
        if (argument == "--camera") {
            draft_.camera_enabled = true;
        } else if (argument == "--camera-source") {
            const auto source = value_after(argument);
            if (source != "rpicam" && source != "gstreamer") {
                throw std::invalid_argument(
                    "--camera-source must be rpicam or gstreamer");
            }
            draft_.camera_backend = source == "rpicam"
                                        ? CameraBackend::rpicam
                                        : CameraBackend::gstreamer;
            explicit_.camera_source = true;
        } else if (argument == "--camera-udp-port") {
            draft_.camera_udp_port =
                parse_number<std::uint16_t>(value_after(argument), argument);
            explicit_.camera_udp_port = true;
        } else if (argument == "--camera-width") {
            draft_.camera_width =
                parse_number<std::uint32_t>(value_after(argument), argument);
            explicit_.camera_width = true;
        } else if (argument == "--camera-height") {
            draft_.camera_height =
                parse_number<std::uint32_t>(value_after(argument), argument);
            explicit_.camera_height = true;
        } else if (argument == "--camera-fps") {
            draft_.camera_fps =
                parse_number<std::uint32_t>(value_after(argument), argument);
            explicit_.camera_fps = true;
        } else if (argument == "--apriltag") {
            draft_.apriltag_enabled = true;
        } else if (argument == "--camera-calibration") {
            draft_.camera_calibration_file = value_after(argument);
        } else if (argument == "--camera-extrinsics") {
            draft_.camera_extrinsics_file = value_after(argument);
        } else if (argument == "--apriltag-size-mm") {
            draft_.apriltag_tag_size_m =
                parse_number<double>(value_after(argument), argument) /
                defaults::kMillimetresPerMetre;
        } else if (argument == "--camera-preview") {
            draft_.camera_preview_enabled = true;
        } else if (argument == "--camera-preview-port") {
            draft_.camera_preview_port =
                parse_number<std::uint16_t>(value_after(argument), argument);
            explicit_.camera_preview_port = true;
        } else if (argument == "--forward-camera-udp-port") {
            draft_.forward_camera_udp_port =
                parse_number<std::uint16_t>(value_after(argument), argument);
        } else if (argument == "--forward-detector-model") {
            draft_.forward_detector_model_file = value_after(argument);
            if (draft_.forward_detector_model_file.empty()) {
                throw std::invalid_argument(
                    "--forward-detector-model cannot be empty");
            }
        } else {
            return false;
        }
        return true;
    }

    bool parse_runtime(const std::string_view argument) {
        if (argument == "--snapshot-ms") {
            draft_.snapshot_interval_ms =
                parse_number<std::uint32_t>(value_after(argument), argument);
        } else if (argument == "--board-types") {
            draft_.board_types_file = value_after(argument);
        } else if (argument == "--json") {
            draft_.json_output = true;
        } else if (argument == "--diagnostic-log") {
            draft_.diagnostic_log_file = value_after(argument);
            explicit_.diagnostic_log_file = true;
        } else if (argument == "--sitl") {
            draft_.sitl_mode = true;
        } else if (argument == "--sim-wind") {
            draft_.simulated_wind = mission::SimulatedWindProfile{
                .speed_m_s =
                    parse_number<double>(value_after(argument), argument),
                .direction_from_deg =
                    parse_number<double>(value_after(argument), argument),
                .turbulence_m_s =
                    parse_number<double>(value_after(argument), argument),
            };
        } else if (argument == "--autonomous") {
            draft_.autonomous = true;
        } else if (argument == "--exit-after-autonomy") {
            draft_.exit_after_autonomy = true;
        } else if (argument == "--interactive") {
            draft_.interactive = true;
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
    LaunchArgumentsDraft draft_;
    ExplicitOptions explicit_;
};

} // namespace

CommandLineOptions parse_command_line(
    const std::vector<std::string_view>& arguments) {
    return ArgumentParser{arguments}.parse();
}

} // namespace onboard_autonomy::operator_interface::cli
