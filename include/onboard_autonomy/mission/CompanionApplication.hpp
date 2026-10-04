#pragma once

#include "onboard_autonomy/mission/AppSnapshot.hpp"
#include "onboard_autonomy/mission/cv/CameraMonitor.hpp"
#include "onboard_autonomy/mission/flight/Transport.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <span>

namespace onboard_autonomy::mission {

struct CompanionApplicationOptions {
    ports::CameraSource* camera_source{nullptr};
    ports::TargetDetector* target_detector{nullptr};
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
    [[nodiscard]] AppSnapshot snapshot(mission::TimePoint now);
    [[nodiscard]] std::optional<ProcessedCameraFrame>
    take_latest_processed_camera_frame();

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard_autonomy::mission
