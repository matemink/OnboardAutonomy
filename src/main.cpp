#include "onboard_autonomy/adapters/ardupilot/BoardTypeCatalog.hpp"
#include "onboard_autonomy/adapters/camera/GStreamerCameraSource.hpp"
#include "onboard_autonomy/adapters/camera/RpicamCameraSource.hpp"
#include "onboard_autonomy/adapters/preview/HttpCameraPreviewServer.hpp"
#include "onboard_autonomy/adapters/transport/TransportFactory.hpp"
#include "onboard_autonomy/adapters/vision/AprilTagTargetDetector.hpp"
#include "onboard_autonomy/adapters/vision/CameraCalibrationLoader.hpp"
#include "onboard_autonomy/adapters/vision/CameraExtrinsicsLoader.hpp"
#include "onboard_autonomy/application/CompanionApplication.hpp"
#include "onboard_autonomy/application/MotionSafetyPolicy.hpp"
#include "onboard_autonomy/presentation/cli/CommandLine.hpp"
#include "onboard_autonomy/presentation/console/ConsoleInput.hpp"
#include "onboard_autonomy/presentation/console/ConsoleView.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

std::atomic_bool keep_running{true};

void handle_signal(int) {
    keep_running = false;
}

class ConsoleSession {
public:
    explicit ConsoleSession(const bool active)
        : active_(active) {
        if (active_) {
            std::cout << "\x1b[2J\x1b[H\x1b[?25l" << std::flush;
        }
    }

    ~ConsoleSession() {
        if (active_) {
            std::cout << "\x1b[?25h\x1b[0m\n" << std::flush;
        }
    }

    ConsoleSession(const ConsoleSession&) = delete;
    ConsoleSession& operator=(const ConsoleSession&) = delete;

private:
    bool active_;
};


std::optional<
    onboard_autonomy::adapters::ardupilot::BoardTypeCatalog
> load_board_type_catalog(
    const onboard_autonomy::presentation::cli::CommandLineOptions&
        options,
    const std::filesystem::path& executable
) {
    std::vector<std::filesystem::path> candidates;
    if (!options.board_types_file.empty()) {
        candidates.emplace_back(options.board_types_file);
    } else {
        candidates.push_back(
            (
                executable.parent_path() /
                ".." /
                "share" /
                "onboard_autonomy" /
                "ardupilot-board-types.txt"
            ).lexically_normal()
        );
        candidates.emplace_back(
            "third_party/ardupilot/board_types.txt"
        );
    }

    for (const auto& candidate : candidates) {
        std::error_code error;
        if (!std::filesystem::is_regular_file(candidate, error)) {
            continue;
        }
        return onboard_autonomy::adapters::ardupilot::
            BoardTypeCatalog::from_file(candidate);
    }

    if (!options.board_types_file.empty()) {
        throw std::runtime_error(
            "board type table not found: " +
            options.board_types_file
        );
    }
    return std::nullopt;
}

std::filesystem::path find_camera_preview_page(
    const std::filesystem::path& executable
) {
    const std::vector<std::filesystem::path> candidates{
        (
            executable.parent_path() /
            ".." /
            "share" /
            "onboard_autonomy" /
            "camera-preview.html"
        ).lexically_normal(),
        "assets/camera-preview/index.html",
    };

    for (const auto& candidate : candidates) {
        std::error_code error;
        if (std::filesystem::is_regular_file(candidate, error)) {
            return candidate;
        }
    }

    throw std::runtime_error(
        "camera preview page was not found"
    );
}

}  // namespace

int main(const int argc, char** argv) {
    try {
        std::vector<std::string_view> arguments;
        arguments.reserve(static_cast<std::size_t>(argc - 1));
        for (int index = 1; index < argc; ++index) {
            arguments.emplace_back(argv[index]);
        }
        const auto options =
            onboard_autonomy::presentation::cli::parse_command_line(
                arguments
            );
        if (options.show_help) {
            std::cout
                << onboard_autonomy::presentation::cli::
                       command_line_help();
            return 0;
        }

        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        auto board_type_catalog = options.json_output
            ? std::optional<
                  onboard_autonomy::adapters::ardupilot::BoardTypeCatalog
              >{}
            : load_board_type_catalog(options, argv[0]);

        const bool has_camera_calibration =
            !options.camera_calibration_file.empty();
        const bool has_camera_extrinsics =
            !options.camera_extrinsics_file.empty();

        const bool motion_requested =
            options.autonomous ||
            options.interactive;
        const auto motion_safety =
            onboard_autonomy::application::evaluate_motion_safety(
                options.sitl_mode
                    ? onboard_autonomy::application::
                          RuntimeEnvironment::sitl
                    : onboard_autonomy::application::
                          RuntimeEnvironment::hardware_or_unknown,
                options.transport ==
                        onboard_autonomy::presentation::cli::
                            TransportBackend::udp
                    ? onboard_autonomy::application::
                          MavlinkTransport::udp
                    : onboard_autonomy::application::
                          MavlinkTransport::serial,
                motion_requested
            );
        if (!motion_safety.configuration_valid) {
            throw std::invalid_argument(
                std::string(motion_safety.reason)
            );
        }

        onboard_autonomy::presentation::console::ConsoleInput console_input{
            options.interactive && !options.json_output
        };
        if (options.interactive && !console_input.active()) {
            throw std::invalid_argument(
                "--interactive requires a live terminal"
            );
        }

        std::unique_ptr<
            onboard_autonomy::application::ports::Transport
        > transport;
        if (options.transport ==
            onboard_autonomy::presentation::cli::
                TransportBackend::udp) {
            transport =
                onboard_autonomy::adapters::transport::make_udp_transport(
                options.udp_bind,
                options.udp_port
            );
        } else {
            transport =
                onboard_autonomy::adapters::transport::make_serial_transport(
                options.serial_device,
                options.baud_rate
            );
        }

        std::unique_ptr<
            onboard_autonomy::application::ports::CameraSource
        > camera_source;
        if (options.camera_enabled) {
            if (options.camera_backend ==
                onboard_autonomy::presentation::cli::
                    CameraBackend::rpicam) {
                camera_source =
                    onboard_autonomy::adapters::camera::
                        make_rpicam_camera_source(
                            {
                                .width = options.camera_width,
                                .height = options.camera_height,
                                .frames_per_second =
                                    options.camera_fps,
                            }
                        );
            } else {
                camera_source =
                    onboard_autonomy::adapters::camera::
                        make_gstreamer_camera_source(
                            {
                                .width = options.camera_width,
                                .height = options.camera_height,
                                .udp_port = options.camera_udp_port,
                            }
                        );
            }
        }
        std::unique_ptr<
            onboard_autonomy::application::ports::TargetDetector
        > target_detector;
        if (options.apriltag_enabled) {
            onboard_autonomy::adapters::vision::
                AprilTagDetectorConfig detector_config;
            if (has_camera_calibration) {
                auto calibration =
                    onboard_autonomy::adapters::vision::
                        CameraCalibrationLoader::from_file(
                            options.camera_calibration_file
                        );
                if (calibration.image_width !=
                        options.camera_width ||
                    calibration.image_height !=
                        options.camera_height) {
                    throw std::invalid_argument(
                        "camera runtime resolution does not match "
                        "calibration"
                    );
                }
                detector_config.pose =
                    onboard_autonomy::adapters::vision::
                        AprilTagPoseConfig{
                            .calibration = std::move(calibration),
                            .tag_size_m =
                                *options.apriltag_tag_size_m,
                        };
            }
            target_detector =
                onboard_autonomy::adapters::vision::
                    make_apriltag_target_detector(
                        std::move(detector_config)
                    );
        }
        std::optional<onboard_autonomy::domain::CameraExtrinsics>
            camera_extrinsics;
        if (has_camera_extrinsics) {
            camera_extrinsics =
                onboard_autonomy::adapters::vision::
                    CameraExtrinsicsLoader::from_file(
                        options.camera_extrinsics_file
                    );
        }
        std::unique_ptr<
            onboard_autonomy::application::ports::CameraPreviewSink
        > camera_preview;
        if (options.camera_preview_enabled) {
            camera_preview =
                onboard_autonomy::adapters::preview::
                    make_http_camera_preview_server(
                        {
                            .bind_address = "0.0.0.0",
                            .port = options.camera_preview_port,
                            .maximum_frames_per_second = 10,
                            .page_file =
                                find_camera_preview_page(argv[0]),
                        }
                    );
        }

        onboard_autonomy::application::CompanionApplication application{
            *transport,
            {
                .flight_startup = {
                    .enabled = options.autonomous,
                    .takeoff_altitude_m = 8.0,
                },
                .autonomy_runtime = {
                    .enabled = options.autonomous,
                },
                .motion_commands_allowed =
                    motion_safety.motion_commands_allowed,
                .camera_source = camera_source.get(),
                .target_detector = target_detector.get(),
                .camera_preview_sink = camera_preview.get(),
                .camera_extrinsics = camera_extrinsics,
                .simulated_wind = options.simulated_wind,
            }
        };
        auto next_snapshot = std::chrono::steady_clock::now();
        bool autonomy_failed = false;
        const auto configured_snapshot_interval =
            std::chrono::milliseconds(options.snapshot_interval_ms);
        const auto snapshot_interval = options.json_output
            ? configured_snapshot_interval
            : std::min(
                  configured_snapshot_interval,
                  std::chrono::milliseconds(100)
              );

        std::cerr << "OnboardAutonomy listening on "
                  << transport->description() << '\n';
        if (camera_preview != nullptr) {
            std::cerr << "Camera preview: http://companionpi.local:"
                      << options.camera_preview_port << "/\n";
        }
        ConsoleSession console_session{!options.json_output};

        while (keep_running) {
            while (const auto key = console_input.poll()) {
                const auto command_time =
                    std::chrono::steady_clock::now();
                if (*key == 's' || *key == 'S') {
                    static_cast<void>(
                        application.request_autonomy_start(
                            command_time
                        )
                    );
                    next_snapshot = command_time;
                } else if (*key == 'q' || *key == 'Q') {
                    keep_running = false;
                }
            }
            if (!keep_running) {
                break;
            }

            application.poll();
            const auto now = std::chrono::steady_clock::now();

            if (now >= next_snapshot) {
                const auto snapshot = application.snapshot(now);
                if (options.json_output) {
                    std::cout
                        << snapshot.to_json()
                        << std::endl;
                } else {
                    std::cout
                        << "\x1b[H"
                        << onboard_autonomy::presentation::console::render_console(
                               snapshot,
                               transport->description(),
                               true,
                               board_type_catalog.has_value()
                                   ? &*board_type_catalog
                                   : nullptr
                           )
                        << std::flush;
                }
                if (options.exit_after_autonomy &&
                    (snapshot.autonomy.phase ==
                         onboard_autonomy::application::
                             AutonomyRuntimePhase::completed ||
                     snapshot.autonomy.phase ==
                         onboard_autonomy::application::
                             AutonomyRuntimePhase::failed)) {
                    autonomy_failed =
                        snapshot.autonomy.phase ==
                        onboard_autonomy::application::
                            AutonomyRuntimePhase::failed;
                    keep_running = false;
                }
                next_snapshot = now + snapshot_interval;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        return autonomy_failed ? 2 : 0;
    } catch (const std::exception& error) {
        std::cerr << "OnboardAutonomy error: " << error.what() << '\n';
        return 1;
    }
}
