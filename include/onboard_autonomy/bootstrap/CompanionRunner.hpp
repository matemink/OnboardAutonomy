#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

namespace onboard_autonomy::mission {
class CompanionApplication;
class AsyncCameraMonitor;
struct AppSnapshot;
} // namespace onboard_autonomy::mission

namespace onboard_autonomy::mission::ports {
class CameraSource;
struct CameraFrame;
} // namespace onboard_autonomy::mission::ports

namespace onboard_autonomy::diagnostics::preview {
class CameraPreviewSink;
}

namespace onboard_autonomy::bootstrap {

class RuntimeSnapshotSink;

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
        mission::CompanionApplication& application,
        mission::AsyncCameraMonitor* forward_camera_monitor,
        RuntimeCommandSource* command_source,
        std::vector<RuntimeSnapshotSink*> snapshot_sinks,
        std::vector<diagnostics::preview::CameraPreviewSink*> preview_sinks);

    [[nodiscard]] int run();

  private:
    void handle_runtime_commands();
    void publish_camera_frames();
    void publish_downward_camera_frame();
    void publish_forward_camera_frame();
    void publish_snapshot(const mission::AppSnapshot& snapshot) const;
    void update_terminal_state(const mission::AppSnapshot& snapshot);

    CompanionRunnerOptions options_;
    mission::CompanionApplication& application_;
    mission::AsyncCameraMonitor* forward_camera_monitor_;
    RuntimeCommandSource* command_source_;
    std::vector<RuntimeSnapshotSink*> snapshot_sinks_;
    std::vector<diagnostics::preview::CameraPreviewSink*> preview_sinks_;
    std::chrono::milliseconds snapshot_interval_;
    std::chrono::steady_clock::time_point next_snapshot_;
    bool autonomy_failed_{};
};

} // namespace onboard_autonomy::bootstrap
