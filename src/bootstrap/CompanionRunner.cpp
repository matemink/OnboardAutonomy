#include "onboard_autonomy/bootstrap/CompanionRunner.hpp"

#include "onboard_autonomy/application/AppSnapshot.hpp"
#include "onboard_autonomy/application/CompanionApplication.hpp"
#include "onboard_autonomy/application/ports/Transport.hpp"
#include "onboard_autonomy/presentation/console/ConsoleView.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace onboard_autonomy::bootstrap {
namespace {

// std::signal requires process-lifetime state accessible to the handler.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
volatile std::sig_atomic_t keep_running = 1;
constexpr auto kMaximumInteractiveRefreshInterval =
    std::chrono::milliseconds{100};
constexpr auto kEventLoopSleep = std::chrono::milliseconds{5};

void handle_signal(int) {
    // Keep signal handling minimal; normal control flow owns all cleanup.
    keep_running = 0;
}

class ConsoleSession {
  public:
    explicit ConsoleSession(const bool active) : active_(active) {
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

bool terminal_phase(const application::AutonomyRuntimePhase phase) {
    return phase == application::AutonomyRuntimePhase::completed ||
           phase == application::AutonomyRuntimePhase::failed;
}

std::chrono::milliseconds snapshot_interval(
    const CompanionRunnerOptions& options) {
    const auto configured =
        std::chrono::milliseconds(options.snapshot_interval_ms);
    return options.json_output
               ? configured
               : std::min(configured, kMaximumInteractiveRefreshInterval);
}

} // namespace

CompanionRunner::CompanionRunner(CompanionRunnerOptions options,
    application::CompanionApplication& application,
    application::ports::Transport& transport,
    const presentation::BoardTypeResolver* board_type_resolver)
    : options_(options),
      console_input_(options.interactive && !options.json_output),
      application_(application), transport_(transport),
      board_type_resolver_(board_type_resolver),
      snapshot_interval_(snapshot_interval(options)),
      next_snapshot_(std::chrono::steady_clock::now()) {
    if (options_.interactive && !console_input_.active()) {
        throw std::invalid_argument("--interactive requires a live terminal");
    }
}

int CompanionRunner::run() {
    keep_running = 1;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    ConsoleSession console_session{!options_.json_output};

    while (keep_running != 0) {
        handle_console_input();
        if (keep_running == 0) {
            break;
        }

        application_.poll();
        const auto now = std::chrono::steady_clock::now();
        if (now < next_snapshot_) {
            std::this_thread::sleep_for(kEventLoopSleep);
            continue;
        }

        const auto snapshot = application_.snapshot(now);
        render_snapshot(snapshot);
        update_terminal_state(snapshot);
        next_snapshot_ = now + snapshot_interval_;
        std::this_thread::sleep_for(kEventLoopSleep);
    }

    return autonomy_failed_ ? 2 : 0;
}

void CompanionRunner::handle_console_input() {
    while (const auto key = console_input_.poll()) {
        const auto command_time = std::chrono::steady_clock::now();
        if (*key == 's' || *key == 'S') {
            static_cast<void>(
                application_.request_autonomy_start(command_time));
            next_snapshot_ = command_time;
        } else if (*key == 'q' || *key == 'Q') {
            keep_running = 0;
        }
    }
}

void CompanionRunner::render_snapshot(
    const application::AppSnapshot& snapshot) const {
    if (options_.json_output) {
        std::cout << snapshot.to_json() << '\n' << std::flush;
        return;
    }

    std::cout << "\x1b[H"
              << presentation::console::render_console(snapshot,
                     transport_.description(),
                     true,
                     board_type_resolver_)
              << std::flush;
}

void CompanionRunner::update_terminal_state(
    const application::AppSnapshot& snapshot) {
    if (!options_.exit_after_autonomy ||
        !terminal_phase(snapshot.autonomy.phase)) {
        return;
    }

    autonomy_failed_ =
        snapshot.autonomy.phase == application::AutonomyRuntimePhase::failed;
    keep_running = 0;
}

} // namespace onboard_autonomy::bootstrap
