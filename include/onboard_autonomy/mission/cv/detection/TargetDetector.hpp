#pragma once

#include "onboard_autonomy/mission/cv/CameraSource.hpp"
#include "onboard_autonomy/mission/cv/detection/TargetObservation.hpp"

#include <string>

namespace onboard_autonomy::mission::ports {

class TargetDetector {
  public:
    virtual ~TargetDetector() = default;

    [[nodiscard]] virtual mission::TargetDetectionBatch detect(
        const CameraFrame& frame) = 0;
    [[nodiscard]] virtual std::string description() const = 0;
};

} // namespace onboard_autonomy::mission::ports
