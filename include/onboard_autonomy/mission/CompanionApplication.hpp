#pragma once

#include "onboard_autonomy/mission/AppSnapshot.hpp"
#include "onboard_autonomy/mission/cv/CameraMonitor.hpp"
#include "onboard_autonomy/mission/flight/Transport.hpp"
#include "onboard_autonomy/mission/cv/extrinsics/TargetTransform.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <span>

namespace onboard_autonomy::mission {

struct CompanionApplicationOptions {
    FlightStartupConfig flight_startup;
    AutonomyRuntimeConfig autonomy_runtime;
    bool motion_commands_allowed{false};
    ports::CameraSource* camera_source{nullptr};
    ports::TargetDetector* target_detector{nullptr};
    std::optional<mission::CameraExtrinsics> camera_extrinsics;
    std::optional<SimulatedWindProfile> simulated_wind;
};

class CompanionApplication {
  public:
    explicit CompanionApplication(ports::Transport& transport,
        CompanionApplicationOptions options = {});
    ~CompanionApplication();

    CompanionApplication(const CompanionApplication&) = delete;
    CompanionApplication& operator=(const CompanionApplication&) = delete;
    CompanionApplication(CompanionApplication&&) = delete;
    CompanionApplication& operator=(CompanionApplication&&) = delete;

    // Production polling captures its scheduling timestamp after the
    // non-blocking transport read. The explicit-time overload keeps tests
    // deterministic.
    void poll();
    void poll(mission::TimePoint now);
    [[nodiscard]] bool request_autonomy_start(mission::TimePoint now);
    [[nodiscard]] AppSnapshot snapshot(mission::TimePoint now);
    [[nodiscard]] std::optional<ProcessedCameraFrame>
    take_latest_processed_camera_frame();
    void update_forward_target_observations(
        std::span<const mission::TargetObservation> observations,
        std::uint32_t frame_width,
        std::uint32_t frame_height,
        std::uint64_t frame_sequence,
        mission::TimePoint now);

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard_autonomy::mission
