#pragma once

#include "onboard_autonomy/mission/cv/CameraSnapshot.hpp"

#include "onboard_autonomy/mission/cv/VisionMonitor.hpp"
#include "onboard_autonomy/mission/cv/CameraSource.hpp"
#include "onboard_autonomy/mission/Clock.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::mission {

struct ProcessedCameraFrame {
    ports::CameraFrame frame;
    mission::TimePoint observed_at;
    std::vector<mission::TargetObservation> targets;
};

class CameraMonitor {
  public:
    explicit CameraMonitor(ports::CameraSource& source,
        ports::TargetDetector* target_detector = nullptr);
    ~CameraMonitor();

    CameraMonitor(const CameraMonitor&) = delete;
    CameraMonitor& operator=(const CameraMonitor&) = delete;
    CameraMonitor(CameraMonitor&&) noexcept;
    CameraMonitor& operator=(CameraMonitor&&) noexcept;

    void poll(mission::TimePoint now);
    [[nodiscard]] CameraSnapshot snapshot(mission::TimePoint now) const;
    [[nodiscard]] std::optional<VisionSnapshot> vision_snapshot(
        mission::TimePoint now) const;
    [[nodiscard]] std::optional<ProcessedCameraFrame>
    take_latest_processed_frame();
    void disable_target_detection();

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard_autonomy::mission
