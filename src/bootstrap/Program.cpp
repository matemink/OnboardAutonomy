#include "onboard_autonomy/bootstrap/Program.hpp"

#include "onboard_autonomy/application/ports/Transport.hpp"
#include "onboard_autonomy/bootstrap/CompanionRunner.hpp"
#include "onboard_autonomy/bootstrap/CompanionSystem.hpp"
#include "onboard_autonomy/presentation/cli/CommandLine.hpp"

#include <iostream>
#include <optional>
#include <string_view>
#include <variant>
#include <vector>

namespace onboard_autonomy::bootstrap {
namespace {

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

    CompanionRunner runner{
        {
            .interactive = operator_interface.interactive,
            .json_output = operator_interface.json_output,
            .exit_after_autonomy = autonomy.exit_when_finished,
            .snapshot_interval_ms = operator_interface.snapshot_interval_ms,
        },
        companion_system.application(),
        companion_system.transport(),
        companion_system.board_type_resolver(),
    };
    return runner.run();
}

} // namespace onboard_autonomy::bootstrap
