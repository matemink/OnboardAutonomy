#pragma once

#include "onboard_autonomy/mission/cv/calibration/CameraCalibration.hpp"
#include "onboard_autonomy/mission/cv/detection/TargetObservation.hpp"

namespace onboard_autonomy::mission::cv {

[[nodiscard]] mission::ImagePoint undistort_image_point(
    const mission::ImagePoint& distorted,
    const mission::CameraCalibration& calibration);

} // namespace onboard_autonomy::mission::cv
