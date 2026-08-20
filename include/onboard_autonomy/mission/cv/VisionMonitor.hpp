#pragma once

#include "onboard_autonomy/mission/cv/tracking/TargetTracker.hpp"
#include "onboard_autonomy/mission/cv/detection/TargetDetector.hpp"
#include "onboard_autonomy/mission/flight/VehicleState.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::mission {

struct VisionSnapshot {
    std::string detector;
    std::uint64_t processed_frames{0};
    std::uint64_t frames_with_targets{0};
    std::uint64_t total_targets{0};
    std::optional<double> latest_processing_ms;
    std::optional<double> average_processing_ms;
    std::optional<double> maximum_processing_ms;
    std::optional<double> last_detection_age_ms;
    std::vector<mission::TargetObservation> latest_targets;
    TargetTrackSnapshot target_track;
};

class VisionMonitor {
  public:
    explicit VisionMonitor(ports::TargetDetector& detector,
        TargetTrackerConfig tracker_config = {});
    ~VisionMonitor();

    VisionMonitor(const VisionMonitor&) = delete;
    VisionMonitor& operator=(const VisionMonitor&) = delete;
    VisionMonitor(VisionMonitor&&) noexcept;
    VisionMonitor& operator=(VisionMonitor&&) noexcept;

    const std::vector<mission::TargetObservation>&
    process(const ports::CameraFrame& frame, mission::TimePoint now);
    [[nodiscard]] VisionSnapshot snapshot(mission::TimePoint now) const;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard_autonomy::mission
