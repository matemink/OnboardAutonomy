#pragma once

#include "onboard_autonomy/mission/cv/tracking/TargetTracker.hpp"
#include "onboard_autonomy/mission/cv/CameraSource.hpp"
#include "onboard_autonomy/mission/cv/detection/TargetObservation.hpp"

#include <span>
#include <string>

namespace onboard_autonomy::diagnostics::preview {

class CameraPreviewSink {
  public:
    virtual ~CameraPreviewSink() = default;

    virtual void publish(const mission::ports::CameraFrame& frame,
        std::span<const mission::TargetObservation> targets,
        const mission::TargetTrackSnapshot& target_track) = 0;

    [[nodiscard]] virtual std::string description() const = 0;
};

} // namespace onboard_autonomy::diagnostics::preview
