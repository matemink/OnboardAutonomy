#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

namespace onboard_autonomy::application {
class CompanionApplication;
struct AppSnapshot;
} // namespace onboard_autonomy::application

namespace onboard_autonomy::application::ports {
class CameraPreviewSink;
class RuntimeSnapshotSink;
} // namespace onboard_autonomy::application::ports

namespace onboard_autonomy::bootstrap {

enum class RuntimeCommand {
    start_autonomy,
    shutdown,
};

class RuntimeCommandSource {
  public:
    virtual ~RuntimeCommandSource() = default;
    [[nodiscard]] virtual std::optional<RuntimeCommand> poll() = 0;
};

struct CompanionRunnerOptions {
    bool exit_after_autonomy{};
    std::uint32_t snapshot_interval_ms{};
};

// Drives input, application polling, and presentation until shutdown.
class CompanionRunner {
  public:
    CompanionRunner(CompanionRunnerOptions options,
        application::CompanionApplication& application,
        RuntimeCommandSource* command_source,
        std::vector<application::ports::RuntimeSnapshotSink*> snapshot_sinks,
        std::vector<application::ports::CameraPreviewSink*> preview_sinks);

    [[nodiscard]] int run();

  private:
    void handle_runtime_commands();
    void publish_camera_frame();
    void publish_snapshot(const application::AppSnapshot& snapshot) const;
    void update_terminal_state(const application::AppSnapshot& snapshot);

    CompanionRunnerOptions options_;
    application::CompanionApplication& application_;
    RuntimeCommandSource* command_source_;
    std::vector<application::ports::RuntimeSnapshotSink*> snapshot_sinks_;
    std::vector<application::ports::CameraPreviewSink*> preview_sinks_;
    std::chrono::milliseconds snapshot_interval_;
    std::chrono::steady_clock::time_point next_snapshot_;
    bool autonomy_failed_{};
};

} // namespace onboard_autonomy::bootstrap
