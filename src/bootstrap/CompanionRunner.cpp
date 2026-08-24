#include "onboard_autonomy/bootstrap/CompanionRunner.hpp"

#include "onboard_autonomy/mission/AppSnapshot.hpp"
#include "onboard_autonomy/mission/CompanionApplication.hpp"
#include "onboard_autonomy/mission/cv/AsyncCameraMonitor.hpp"
#include "onboard_autonomy/mission/cv/CameraSource.hpp"
#include "onboard_autonomy/diagnostics/preview/CameraPreviewSink.hpp"
#include "onboard_autonomy/bootstrap/RuntimeSnapshotSink.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <span>
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

bool terminal_phase(const mission::AutonomyRuntimePhase phase) {
    return phase == mission::AutonomyRuntimePhase::completed ||
           phase == mission::AutonomyRuntimePhase::failed;
}

} // namespace

CompanionRunner::CompanionRunner(CompanionRunnerOptions options,
    mission::CompanionApplication& application,
    mission::AsyncCameraMonitor* forward_camera_monitor,
    RuntimeCommandSource* command_source,
    std::vector<bootstrap::RuntimeSnapshotSink*> snapshot_sinks,
    std::vector<diagnostics::preview::CameraPreviewSink*> preview_sinks)
    : options_(options), application_(application),
      forward_camera_monitor_(forward_camera_monitor),
      command_source_(command_source),
      snapshot_sinks_(std::move(snapshot_sinks)),
      preview_sinks_(std::move(preview_sinks)),
      snapshot_interval_(options.snapshot_interval_ms),
      next_snapshot_(std::chrono::steady_clock::now()) {
    if (snapshot_sinks_.empty()) {
        throw std::invalid_argument(
            "at least one runtime snapshot sink is required");
    }
}

int CompanionRunner::run() {
    keep_running = 1;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    while (keep_running != 0) {
        handle_runtime_commands();
        if (keep_running == 0) {
            break;
        }

        application_.poll();
        publish_camera_frames();
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

void CompanionRunner::handle_runtime_commands() {
    if (command_source_ == nullptr) {
        return;
    }
    while (const auto command = command_source_->poll()) {
        const auto command_time = std::chrono::steady_clock::now();
        if (*command == RuntimeCommand::start_autonomy) {
            static_cast<void>(
                application_.request_autonomy_start(command_time));
            next_snapshot_ = command_time;
        } else if (*command == RuntimeCommand::shutdown) {
            keep_running = 0;
        }
    }
}

void CompanionRunner::publish_camera_frames() {
    publish_downward_camera_frame();
    publish_forward_camera_frame();
}

void CompanionRunner::publish_downward_camera_frame() {
    auto processed = application_.take_latest_processed_camera_frame();
    if (!processed.has_value()) {
        return;
    }
    for (auto* sink : preview_sinks_) {
        if (sink != nullptr) {
            sink->publish(diagnostics::preview::CameraPreviewStream::downward,
                processed->frame,
                processed->targets,
                processed->target_track);
        }
    }
}

void CompanionRunner::publish_forward_camera_frame() {
    if (forward_camera_monitor_ == nullptr) {
        return;
    }
    if (const auto error = forward_camera_monitor_->take_latest_error()) {
        std::cerr << "Forward camera detection disabled: " << *error << '\n';
    }
    auto processed = forward_camera_monitor_->take_latest_processed_frame();
    if (!processed.has_value()) {
        return;
    }
    application_.update_forward_target_observations(processed->targets,
        processed->frame.width,
        processed->frame.height,
        processed->frame.sequence,
        std::chrono::steady_clock::now());
    for (auto* sink : preview_sinks_) {
        if (sink != nullptr) {
            sink->publish(diagnostics::preview::CameraPreviewStream::forward,
                processed->frame,
                processed->targets,
                processed->target_track);
        }
    }
}

void CompanionRunner::publish_snapshot(
    const mission::AppSnapshot& snapshot) const {
    const auto recorded_at = std::chrono::system_clock::now();
    for (auto* sink : snapshot_sinks_) {
        if (sink != nullptr) {
            sink->consume(snapshot, recorded_at);
        }
    }
}

void CompanionRunner::update_terminal_state(
    const mission::AppSnapshot& snapshot) {
    if (!options_.exit_after_autonomy ||
        !terminal_phase(snapshot.autonomy.phase)) {
        return;
    }

    autonomy_failed_ =
        snapshot.autonomy.phase == mission::AutonomyRuntimePhase::failed;
    keep_running = 0;
}

} // namespace onboard_autonomy::bootstrap
