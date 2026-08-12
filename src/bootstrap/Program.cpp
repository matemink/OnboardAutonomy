#include "onboard_autonomy/bootstrap/Program.hpp"

#include "onboard_autonomy/application/AppSnapshot.hpp"
#include "onboard_autonomy/application/CompanionApplication.hpp"
#include "onboard_autonomy/bootstrap/RuntimeAssembly.hpp"
#include "onboard_autonomy/presentation/cli/CommandLine.hpp"
#include "onboard_autonomy/presentation/console/ConsoleInput.hpp"
#include "onboard_autonomy/presentation/console/ConsoleView.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

namespace onboard_autonomy::bootstrap {
namespace {

volatile std::sig_atomic_t keep_running = 1;

void handle_signal(int) {
    // Keep signal handling minimal; normal control flow owns all cleanup.
    keep_running = 0;
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
        // Always restore the cursor and colors, including exception exits.
        if (active_) {
            std::cout << "\x1b[?25h\x1b[0m\n" << std::flush;
        }
    }

    ConsoleSession(const ConsoleSession&) = delete;
    ConsoleSession& operator=(const ConsoleSession&) = delete;

private:
    bool active_;
};

std::vector<std::string_view> command_line_arguments(
    const int argc,
    char** argv
) {
    std::vector<std::string_view> arguments;
    arguments.reserve(static_cast<std::size_t>(argc - 1));
    for (int index = 1; index < argc; ++index) {
        arguments.emplace_back(argv[index]);
    }
    return arguments;
}

bool terminal_phase(const application::AutonomyRuntimePhase phase) {
    return phase == application::AutonomyRuntimePhase::completed ||
        phase == application::AutonomyRuntimePhase::failed;
}

void handle_console_input(
    presentation::console::ConsoleInput& input,
    application::CompanionApplication& application,
    std::chrono::steady_clock::time_point& next_snapshot
) {
    while (const auto key = input.poll()) {
        const auto command_time = std::chrono::steady_clock::now();
        if (*key == 's' || *key == 'S') {
            static_cast<void>(
                application.request_autonomy_start(command_time)
            );
            next_snapshot = command_time;
        } else if (*key == 'q' || *key == 'Q') {
            keep_running = 0;
        }
    }
}

}  // namespace

int run_program(const int argc, char** argv) {
    const auto options = presentation::cli::parse_command_line(
        command_line_arguments(argc, argv)
    );
    if (options.show_help) {
        std::cout << presentation::cli::command_line_help();
        return 0;
    }

    keep_running = 1;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    presentation::console::ConsoleInput console_input{
        options.interactive && !options.json_output
    };
    if (options.interactive && !console_input.active()) {
        throw std::invalid_argument(
            "--interactive requires a live terminal"
        );
    }

    RuntimeAssembly runtime{options, argv[0]};
    auto& application = runtime.application();
    auto& transport = runtime.transport();

    const auto configured_snapshot_interval =
        std::chrono::milliseconds(options.snapshot_interval_ms);
    const auto snapshot_interval = options.json_output
        ? configured_snapshot_interval
        : std::min(
              configured_snapshot_interval,
              std::chrono::milliseconds(100)
          );
    auto next_snapshot = std::chrono::steady_clock::now();
    bool autonomy_failed = false;

    std::cerr << "OnboardAutonomy listening on "
              << transport.description() << '\n';
    if (options.camera_preview_enabled) {
        std::cerr << "Camera preview: http://companionpi.local:"
                  << options.camera_preview_port << "/\n";
    }
    ConsoleSession console_session{!options.json_output};

    while (keep_running != 0) {
        handle_console_input(console_input, application, next_snapshot);
        if (keep_running == 0) {
            break;
        }

        application.poll();
        const auto now = std::chrono::steady_clock::now();
        if (now < next_snapshot) {
            // Keep polling MAVLink and input between slower UI refreshes.
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        const auto snapshot = application.snapshot(now);
        if (options.json_output) {
            std::cout << snapshot.to_json() << '\n' << std::flush;
        } else {
            std::cout
                << "\x1b[H"
                << presentation::console::render_console(
                       snapshot,
                       transport.description(),
                       true,
                       runtime.board_type_resolver()
                   )
                << std::flush;
        }

        if (options.exit_after_autonomy &&
            terminal_phase(snapshot.autonomy.phase)) {
            autonomy_failed = snapshot.autonomy.phase ==
                application::AutonomyRuntimePhase::failed;
            keep_running = 0;
        }
        next_snapshot = now + snapshot_interval;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return autonomy_failed ? 2 : 0;
}

}  // namespace onboard_autonomy::bootstrap
