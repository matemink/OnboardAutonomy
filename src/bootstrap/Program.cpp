#include "onboard_autonomy/bootstrap/Program.hpp"

#include "onboard_autonomy/adapters/ardupilot/BoardTypeCatalog.hpp"
#include "onboard_autonomy/adapters/preview/HttpCameraPreviewServer.hpp"
#include "onboard_autonomy/application/ports/CameraPreviewSink.hpp"
#include "onboard_autonomy/application/ports/RuntimeSnapshotSink.hpp"
#include "onboard_autonomy/application/ports/Transport.hpp"
#include "onboard_autonomy/bootstrap/CompanionRunner.hpp"
#include "onboard_autonomy/bootstrap/MissionRuntime.hpp"
#include "onboard_autonomy/diagnostics/logging/JsonDiagnosticSink.hpp"
#include "onboard_autonomy/presentation/cli/CommandLine.hpp"
#include "onboard_autonomy/presentation/console/ConsoleInput.hpp"
#include "onboard_autonomy/presentation/console/ConsoleSnapshotSink.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace onboard_autonomy::bootstrap {
namespace {

constexpr std::uint32_t kMaximumConsoleRefreshIntervalMs = 100;

using BoardTypeCatalog = adapters::ardupilot::BoardTypeCatalog;
using CameraPreviewSink = application::ports::CameraPreviewSink;
using RuntimeSnapshotSink = application::ports::RuntimeSnapshotSink;

std::vector<std::string_view> command_line_arguments(const int argc,
    char** argv) {
    std::vector<std::string_view> arguments;
    arguments.reserve(static_cast<std::size_t>(argc - 1));
    for (int index = 1; index < argc; ++index) {
        arguments.emplace_back(argv[index]);
    }
    return arguments;
}

const presentation::cli::AutonomyOptions& autonomy_options(
    const presentation::cli::CommandLineOptions& options) {
    return std::visit(
        [](const auto& launch) -> const presentation::cli::AutonomyOptions& {
            return launch.autonomy;
        },
        options);
}

const presentation::cli::OperatorInterfaceOptions& operator_options(
    const presentation::cli::CommandLineOptions& options) {
    return std::visit(
        [](const auto& launch)
            -> const presentation::cli::OperatorInterfaceOptions& {
            return launch.operator_interface;
        },
        options);
}

const presentation::cli::DiagnosticsOptions& diagnostics_options(
    const presentation::cli::CommandLineOptions& options) {
    return std::visit(
        [](const auto& launch) -> const presentation::cli::DiagnosticsOptions& {
            return launch.diagnostics;
        },
        options);
}

MissionConnection make_mission_connection(
    const presentation::cli::MavlinkConnectionOptions& connection) {
    return std::visit(
        [](const auto& configured) -> MissionConnection {
            using Connection = std::decay_t<decltype(configured)>;
            if constexpr (std::is_same_v<Connection,
                              presentation::cli::UdpConnectionOptions>) {
                return UdpMissionConnection{
                    .bind_address = configured.bind_address,
                    .port = configured.port,
                };
            } else {
                return SerialMissionConnection{
                    .device = configured.device,
                    .baud_rate = configured.baud_rate,
                };
            }
        },
        connection);
}

MissionCameraSource make_mission_camera_source(
    const presentation::cli::CameraSourceOptions& source) {
    return std::visit(
        [](const auto& configured) -> MissionCameraSource {
            using Source = std::decay_t<decltype(configured)>;
            if constexpr (std::is_same_v<Source,
                              presentation::cli::RpicamOptions>) {
                return RpicamMissionSource{
                    .frames_per_second = configured.frames_per_second,
                };
            } else {
                return GStreamerMissionSource{
                    .udp_port = configured.udp_port,
                };
            }
        },
        source);
}

std::optional<MissionCameraConfig> make_mission_camera(
    const std::optional<presentation::cli::CameraOptions>& configured) {
    if (!configured.has_value()) {
        return std::nullopt;
    }

    std::optional<AprilTagMissionConfig> apriltag;
    if (configured->apriltag.has_value()) {
        apriltag = AprilTagMissionConfig{
            .calibration_file = configured->apriltag->calibration_file,
            .extrinsics_file = configured->apriltag->extrinsics_file,
            .tag_size_m = configured->apriltag->tag_size_m,
        };
    }
    return MissionCameraConfig{
        .source = make_mission_camera_source(configured->source),
        .apriltag = std::move(apriltag),
        .frame_width = configured->frame_width,
        .frame_height = configured->frame_height,
    };
}

MissionRuntimeConfig make_mission_runtime_config(
    const presentation::cli::HardwareLaunchOptions& options) {
    return {
        .connection = make_mission_connection(options.connection),
        .camera = make_mission_camera(options.camera),
        .simulated_wind = std::nullopt,
        .environment = MissionEnvironment::hardware,
        .autonomous = options.autonomy.enabled,
        .motion_commands_requested =
            options.autonomy.enabled || options.operator_interface.interactive,
    };
}

MissionRuntimeConfig make_mission_runtime_config(
    const presentation::cli::SimulationLaunchOptions& options) {
    return {
        .connection =
            UdpMissionConnection{
                .bind_address = options.connection.bind_address,
                .port = options.connection.port,
            },
        .camera = make_mission_camera(options.camera),
        .simulated_wind = options.wind,
        .environment = MissionEnvironment::simulation,
        .autonomous = options.autonomy.enabled,
        .motion_commands_requested =
            options.autonomy.enabled || options.operator_interface.interactive,
    };
}

MissionRuntimeConfig make_mission_runtime_config(
    const presentation::cli::CommandLineOptions& options) {
    return std::visit(
        [](const auto& launch) { return make_mission_runtime_config(launch); },
        options);
}

std::unique_ptr<BoardTypeCatalog> load_board_type_catalog(
    const presentation::cli::OperatorInterfaceOptions& options,
    const std::filesystem::path& executable) {
    if (options.json_output) {
        return nullptr;
    }

    std::vector<std::filesystem::path> candidates;
    if (!options.board_types_file.empty()) {
        candidates.emplace_back(options.board_types_file);
    } else {
        candidates.push_back((executable.parent_path() / ".." / "share" /
                              "onboard_autonomy" / "ardupilot-board-types.txt")
                                 .lexically_normal());
        candidates.emplace_back("third_party/ardupilot/board_types.txt");
    }

    for (const auto& candidate : candidates) {
        std::error_code error;
        if (std::filesystem::is_regular_file(candidate, error)) {
            return std::make_unique<BoardTypeCatalog>(
                BoardTypeCatalog::from_file(candidate));
        }
    }

    if (!options.board_types_file.empty()) {
        throw std::runtime_error(
            "board type table not found: " + options.board_types_file);
    }
    return nullptr;
}

std::filesystem::path find_camera_preview_page(
    const std::filesystem::path& executable) {
    const std::vector<std::filesystem::path> candidates{
        (executable.parent_path() / ".." / "share" / "onboard_autonomy" /
            "camera-preview.html")
            .lexically_normal(),
        "assets/camera-preview/index.html",
    };
    for (const auto& candidate : candidates) {
        std::error_code error;
        if (std::filesystem::is_regular_file(candidate, error)) {
            return candidate;
        }
    }
    throw std::runtime_error("camera preview page was not found");
}

std::unique_ptr<CameraPreviewSink> make_camera_preview(
    const presentation::cli::DiagnosticsOptions& options,
    const std::filesystem::path& executable) {
    if (!options.camera_preview.has_value()) {
        return nullptr;
    }
    return adapters::preview::make_http_camera_preview_server({
        .bind_address = "0.0.0.0",
        .port = options.camera_preview->port,
        .maximum_frames_per_second = adapters::preview::
            HttpCameraPreviewConfig::kDefaultMaximumFramesPerSecond,
        .page_file = find_camera_preview_page(executable),
    });
}

std::uint32_t snapshot_interval_ms(
    const presentation::cli::OperatorInterfaceOptions& options) {
    return options.json_output ? options.snapshot_interval_ms
                               : std::min(options.snapshot_interval_ms,
                                     kMaximumConsoleRefreshIntervalMs);
}

std::vector<std::unique_ptr<RuntimeSnapshotSink>> make_snapshot_sinks(
    const presentation::cli::OperatorInterfaceOptions& operator_interface,
    const presentation::cli::DiagnosticsOptions& diagnostics,
    const application::ports::Transport& transport,
    const presentation::BoardTypeResolver* board_type_resolver) {
    std::vector<std::unique_ptr<RuntimeSnapshotSink>> sinks;
    if (operator_interface.json_output) {
        sinks.push_back(
            std::make_unique<diagnostics::logging::JsonDiagnosticSink>(
                std::cout));
    } else {
        sinks.push_back(
            std::make_unique<presentation::console::ConsoleSnapshotSink>(
                std::cout,
                transport.description(),
                board_type_resolver));
    }
    if (!diagnostics.log_file.empty()) {
        sinks.push_back(
            std::make_unique<diagnostics::logging::JsonDiagnosticSink>(
                diagnostics.log_file));
    }
    return sinks;
}

template <typename Sink>
std::vector<Sink*> sink_pointers(
    const std::vector<std::unique_ptr<Sink>>& sinks) {
    std::vector<Sink*> result;
    result.reserve(sinks.size());
    std::transform(sinks.begin(),
        sinks.end(),
        std::back_inserter(result),
        [](const auto& sink) { return sink.get(); });
    return result;
}

std::vector<CameraPreviewSink*> preview_sinks(CameraPreviewSink* preview) {
    return preview == nullptr ? std::vector<CameraPreviewSink*>{}
                              : std::vector<CameraPreviewSink*>{preview};
}

class ConsoleCommandSource final : public RuntimeCommandSource {
  public:
    explicit ConsoleCommandSource(const bool enabled) : input_(enabled) {
        if (enabled && !input_.active()) {
            throw std::invalid_argument(
                "--interactive requires a live terminal");
        }
    }

    [[nodiscard]] std::optional<RuntimeCommand> poll() override {
        while (const auto key = input_.poll()) {
            if (*key == 's' || *key == 'S') {
                return RuntimeCommand::start_autonomy;
            }
            if (*key == 'q' || *key == 'Q') {
                return RuntimeCommand::shutdown;
            }
        }
        return std::nullopt;
    }

  private:
    presentation::console::ConsoleInput input_;
};

} // namespace

int run_program(const int argc, char** argv) {
    const auto options = presentation::cli::parse_command_line(
        command_line_arguments(argc, argv));
    const auto& autonomy = autonomy_options(options);
    const auto& operator_interface = operator_options(options);
    const auto& diagnostics = diagnostics_options(options);
    const std::filesystem::path executable{argv[0]};

    MissionRuntime mission{make_mission_runtime_config(options)};
    auto board_types = load_board_type_catalog(operator_interface, executable);
    auto snapshot_sinks = make_snapshot_sinks(operator_interface,
        diagnostics,
        mission.transport(),
        board_types.get());
    auto camera_preview = make_camera_preview(diagnostics, executable);
    ConsoleCommandSource operator_commands{operator_interface.interactive};

    std::cerr << "OnboardAutonomy listening on "
              << mission.transport().description() << '\n';
    if (diagnostics.camera_preview.has_value()) {
        std::cerr << "Camera preview: http://companionpi.local:"
                  << diagnostics.camera_preview->port << "/\n";
    }

    CompanionRunner runner{
        {
            .exit_after_autonomy = autonomy.exit_when_finished,
            .snapshot_interval_ms = snapshot_interval_ms(operator_interface),
        },
        mission.application(),
        operator_interface.interactive ? &operator_commands : nullptr,
        sink_pointers(snapshot_sinks),
        preview_sinks(camera_preview.get()),
    };
    return runner.run();
}

} // namespace onboard_autonomy::bootstrap
