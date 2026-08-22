#pragma once

#include "onboard_autonomy/mission/cv/CameraMonitor.hpp"

#include <memory>
#include <optional>

namespace onboard_autonomy::mission {

class AsyncCameraMonitor {
  public:
    explicit AsyncCameraMonitor(ports::CameraSource& source,
        ports::TargetDetector* target_detector = nullptr);
    ~AsyncCameraMonitor();

    AsyncCameraMonitor(const AsyncCameraMonitor&) = delete;
    AsyncCameraMonitor& operator=(const AsyncCameraMonitor&) = delete;
    AsyncCameraMonitor(AsyncCameraMonitor&&) = delete;
    AsyncCameraMonitor& operator=(AsyncCameraMonitor&&) = delete;

    [[nodiscard]] std::optional<ProcessedCameraFrame>
    take_latest_processed_frame();

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard_autonomy::mission
