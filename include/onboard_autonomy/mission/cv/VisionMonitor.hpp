#pragma once

#include "onboard_autonomy/mission/cv/VisionSnapshot.hpp"

#include "onboard_autonomy/mission/cv/detection/TargetDetector.hpp"
#include "onboard_autonomy/mission/Clock.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::mission {

class VisionMonitor {
  public:
    explicit VisionMonitor(ports::TargetDetector& detector);
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
