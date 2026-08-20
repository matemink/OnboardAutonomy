#include "onboard_autonomy/bootstrap/CompanionRunner.hpp"

#include "onboard_autonomy/application/AppSnapshot.hpp"
#include "onboard_autonomy/application/CompanionApplication.hpp"
#include "onboard_autonomy/application/ports/RuntimeSnapshotSink.hpp"

#include <chrono>
#include <csignal>
#include <stdexcept>
#include <thread>
#include <utility>

namespace onboard_autonomy::bootstrap {
namespace {

// std::signal requires process-lifetime state accessible to the handler.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
volatile std::sig_atomic_t keep_running = 1;
constexpr auto kEventLoopSleep = std::chrono::milliseconds{5};

void handle_signal(int) {
    // Keep signal handling minimal; normal control flow owns all cleanup.
    keep_running = 0;
}

bool terminal_phase(const application::AutonomyRuntimePhase phase) {
    return phase == application::AutonomyRuntimePhase::completed ||
           phase == application::AutonomyRuntimePhase::failed;
}

} // namespace

CompanionRunner::CompanionRunner(CompanionRunnerOptions options,
    application::CompanionApplication& application,
    std::vector<application::ports::RuntimeSnapshotSink*> snapshot_sinks)
    : options_(options), console_input_(options.interactive),
      application_(application), snapshot_sinks_(std::move(snapshot_sinks)),
      snapshot_interval_(options.snapshot_interval_ms),
      next_snapshot_(std::chrono::steady_clock::now()) {
    if (snapshot_sinks_.empty()) {
        throw std::invalid_argument(
            "at least one runtime snapshot sink is required");
    }
    if (options_.interactive && !console_input_.active()) {
        throw std::invalid_argument("--interactive requires a live terminal");
    }
}

int CompanionRunner::run() {
    keep_running = 1;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
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
        publish_snapshot(snapshot);
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

void CompanionRunner::publish_snapshot(
    const application::AppSnapshot& snapshot) const {
    const auto recorded_at = std::chrono::system_clock::now();
    for (auto* sink : snapshot_sinks_) {
        if (sink != nullptr) {
            sink->consume(snapshot, recorded_at);
        }
    }
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
