#pragma once

#include "onboard_autonomy/application/AppSnapshot.hpp"
#include "onboard_autonomy/application/ports/CameraPreviewSink.hpp"
#include "onboard_autonomy/application/ports/Transport.hpp"
#include "onboard_autonomy/domain/TargetTransform.hpp"

#include <memory>
#include <optional>

namespace onboard_autonomy::application {

struct CompanionApplicationOptions {
    FlightStartupConfig flight_startup;
    AutonomyRuntimeConfig autonomy_runtime;
    bool motion_commands_allowed{false};
    ports::CameraSource* camera_source{nullptr};
    ports::TargetDetector* target_detector{nullptr};
    ports::CameraPreviewSink* camera_preview_sink{nullptr};
    std::optional<domain::CameraExtrinsics> camera_extrinsics;
    std::optional<SimulatedWindProfile> simulated_wind;
};

class CompanionApplication {
public:
    explicit CompanionApplication(
        ports::Transport& transport,
        CompanionApplicationOptions options = {}
    );
    ~CompanionApplication();

    CompanionApplication(const CompanionApplication&) = delete;
    CompanionApplication& operator=(const CompanionApplication&) = delete;
    CompanionApplication(CompanionApplication&&) = delete;
    CompanionApplication& operator=(CompanionApplication&&) = delete;

    // Production polling captures its scheduling timestamp after the
    // non-blocking transport read. The explicit-time overload keeps tests
    // deterministic.
    void poll();
    void poll(domain::TimePoint now);
    [[nodiscard]] bool request_autonomy_start(domain::TimePoint now);
    [[nodiscard]] AppSnapshot snapshot(domain::TimePoint now);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace onboard_autonomy::application
