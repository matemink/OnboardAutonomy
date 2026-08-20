#pragma once

#include "onboard_autonomy/presentation/console/ConsoleInput.hpp"

#include <chrono>
#include <cstdint>
#include <vector>

namespace onboard_autonomy::application {
class CompanionApplication;
struct AppSnapshot;
} // namespace onboard_autonomy::application

namespace onboard_autonomy::application::ports {
class RuntimeSnapshotSink;
}

namespace onboard_autonomy::bootstrap {

struct CompanionRunnerOptions {
    bool interactive{};
    bool exit_after_autonomy{};
    std::uint32_t snapshot_interval_ms{};
};

// Drives input, application polling, and presentation until shutdown.
class CompanionRunner {
  public:
    CompanionRunner(CompanionRunnerOptions options,
        application::CompanionApplication& application,
        std::vector<application::ports::RuntimeSnapshotSink*> snapshot_sinks);

    [[nodiscard]] int run();

  private:
    void handle_console_input();
    void publish_snapshot(const application::AppSnapshot& snapshot) const;
    void update_terminal_state(const application::AppSnapshot& snapshot);

    CompanionRunnerOptions options_;
    presentation::console::ConsoleInput console_input_;
    application::CompanionApplication& application_;
    std::vector<application::ports::RuntimeSnapshotSink*> snapshot_sinks_;
    std::chrono::milliseconds snapshot_interval_;
    std::chrono::steady_clock::time_point next_snapshot_;
    bool autonomy_failed_{};
};

} // namespace onboard_autonomy::bootstrap
