#include "onboard_autonomy/mission/cv/detection/CameraGeometry.hpp"

#include <cmath>
#include <stdexcept>

namespace onboard_autonomy::mission::cv {

mission::ImagePoint undistort_image_point(const mission::ImagePoint& distorted,
    const mission::CameraCalibration& calibration) {
    mission::validate_camera_calibration(calibration);
    if (!std::isfinite(distorted.x_px) || !std::isfinite(distorted.y_px)) {
        throw std::invalid_argument("distorted image point must be finite");
    }

    const double distorted_x =
        (distorted.x_px - calibration.cx_px) / calibration.fx_px;
    const double distorted_y =
        (distorted.y_px - calibration.cy_px) / calibration.fy_px;
    double x = distorted_x;
    double y = distorted_y;

    const auto [k1, k2, p1, p2, k3] = calibration.distortion;
    constexpr int iteration_count = 10;
    constexpr double minimum_radial_scale = 1.0e-12;
    for (int iteration = 0; iteration < iteration_count; ++iteration) {
        const double radius_squared = x * x + y * y;
        const double radius_fourth = radius_squared * radius_squared;
        const double radius_sixth = radius_fourth * radius_squared;
        const double radial_scale =
            1.0 + k1 * radius_squared + k2 * radius_fourth + k3 * radius_sixth;
        if (!std::isfinite(radial_scale) ||
            std::abs(radial_scale) < minimum_radial_scale) {
            throw std::runtime_error("camera distortion cannot be inverted");
        }

        const double tangential_x =
            2.0 * p1 * x * y + p2 * (radius_squared + 2.0 * x * x);
        const double tangential_y =
            p1 * (radius_squared + 2.0 * y * y) + 2.0 * p2 * x * y;
        x = (distorted_x - tangential_x) / radial_scale;
        y = (distorted_y - tangential_y) / radial_scale;
    }

    const mission::ImagePoint result{
        .x_px = calibration.fx_px * x + calibration.cx_px,
        .y_px = calibration.fy_px * y + calibration.cy_px,
    };
    if (!std::isfinite(result.x_px) || !std::isfinite(result.y_px)) {
        throw std::runtime_error(
            "camera undistortion produced a non-finite point");
    }
    return result;
}

} // namespace onboard_autonomy::mission::cv
