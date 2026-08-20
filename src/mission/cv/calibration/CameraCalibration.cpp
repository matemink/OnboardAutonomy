#include "onboard_autonomy/mission/cv/calibration/CameraCalibration.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace onboard_autonomy::mission {

void validate_camera_calibration(const CameraCalibration& calibration) {
    const bool finite_intrinsics =
        std::isfinite(calibration.fx_px) && std::isfinite(calibration.fy_px) &&
        std::isfinite(calibration.cx_px) && std::isfinite(calibration.cy_px);
    const bool finite_distortion = std::ranges::all_of(calibration.distortion,
        [](const double coefficient) { return std::isfinite(coefficient); });

    if (calibration.camera_model.empty() || calibration.focus_mode.empty() ||
        calibration.lens_position.empty() || calibration.image_width == 0U ||
        calibration.image_height == 0U || !finite_intrinsics ||
        !finite_distortion || calibration.fx_px <= 0.0 ||
        calibration.fy_px <= 0.0 || calibration.cx_px < 0.0 ||
        calibration.cy_px < 0.0 ||
        calibration.cx_px >= static_cast<double>(calibration.image_width) ||
        calibration.cy_px >= static_cast<double>(calibration.image_height)) {
        throw std::invalid_argument("invalid camera calibration");
    }
}

} // namespace onboard_autonomy::mission
