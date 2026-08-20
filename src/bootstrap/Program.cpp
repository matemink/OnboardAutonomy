#include "onboard_autonomy/bootstrap/Program.hpp"

#include "onboard_autonomy/application/ports/RuntimeSnapshotSink.hpp"
#include "onboard_autonomy/application/ports/Transport.hpp"
#include "onboard_autonomy/bootstrap/CompanionRunner.hpp"
#include "onboard_autonomy/bootstrap/CompanionSystem.hpp"
#include "onboard_autonomy/diagnostics/logging/JsonDiagnosticSink.hpp"
#include "onboard_autonomy/presentation/cli/CommandLine.hpp"
#include "onboard_autonomy/presentation/console/ConsoleSnapshotSink.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <string_view>
#include <variant>
#include <vector>

namespace onboard_autonomy::bootstrap {
namespace {

constexpr std::uint32_t kMaximumConsoleRefreshIntervalMs = 100;

std::vector<std::string_view> command_line_arguments(const int argc,
    char** argv) {
    std::vector<std::string_view> arguments;
    arguments.reserve(static_cast<std::size_t>(argc - 1));
    for (int index = 1; index < argc; ++index) {
        arguments.emplace_back(argv[index]);
    }
    return arguments;
}

const std::optional<presentation::cli::CameraOptions>& camera_options(
    const presentation::cli::CommandLineOptions& options) {
    return std::visit(
        [](const auto& launch)
            -> const std::optional<presentation::cli::CameraOptions>& {
            return launch.camera;
        },
        options);
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

std::uint32_t snapshot_interval_ms(
    const presentation::cli::OperatorInterfaceOptions& options) {
    return options.json_output ? options.snapshot_interval_ms
                               : std::min(options.snapshot_interval_ms,
                                     kMaximumConsoleRefreshIntervalMs);
}

using RuntimeSnapshotSink = application::ports::RuntimeSnapshotSink;

std::vector<std::unique_ptr<RuntimeSnapshotSink>> make_snapshot_sinks(
    const presentation::cli::OperatorInterfaceOptions& options,
    CompanionSystem& system) {
    std::vector<std::unique_ptr<RuntimeSnapshotSink>> sinks;
    if (options.json_output) {
        sinks.push_back(
            std::make_unique<diagnostics::logging::JsonDiagnosticSink>(
                std::cout));
    } else {
        sinks.push_back(
            std::make_unique<presentation::console::ConsoleSnapshotSink>(
                std::cout,
                system.transport().description(),
                system.board_type_resolver()));
    }
    if (!options.diagnostic_log_file.empty()) {
        sinks.push_back(
            std::make_unique<diagnostics::logging::JsonDiagnosticSink>(
                options.diagnostic_log_file));
    }
    return sinks;
}

std::vector<RuntimeSnapshotSink*> sink_pointers(
    const std::vector<std::unique_ptr<RuntimeSnapshotSink>>& sinks) {
    std::vector<RuntimeSnapshotSink*> result;
    result.reserve(sinks.size());
    std::transform(sinks.begin(),
        sinks.end(),
        std::back_inserter(result),
        [](const auto& sink) { return sink.get(); });
    return result;
}

} // namespace

int run_program(const int argc, char** argv) {
    const auto options = presentation::cli::parse_command_line(
        command_line_arguments(argc, argv));
    const auto& camera = camera_options(options);
    const auto& autonomy = autonomy_options(options);
    const auto& operator_interface = operator_options(options);

    CompanionSystem companion_system{options, argv[0]};
    std::cerr << "OnboardAutonomy listening on "
              << companion_system.transport().description() << '\n';
    if (camera.has_value() && camera->preview.has_value()) {
        std::cerr << "Camera preview: http://companionpi.local:"
                  << camera->preview->port << "/\n";
    }

    auto snapshot_sinks =
        make_snapshot_sinks(operator_interface, companion_system);
    CompanionRunner runner{
        {
            .interactive = operator_interface.interactive,
            .exit_after_autonomy = autonomy.exit_when_finished,
            .snapshot_interval_ms = snapshot_interval_ms(operator_interface),
        },
        companion_system.application(),
        sink_pointers(snapshot_sinks),
    };
    return runner.run();
}

} // namespace onboard_autonomy::bootstrap
