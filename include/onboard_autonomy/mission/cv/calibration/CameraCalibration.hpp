#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace onboard_autonomy::mission {

struct CameraCalibration {
    static constexpr std::size_t kDistortionCoefficientCount = 5;

    std::string camera_model;
    std::uint32_t image_width{0};
    std::uint32_t image_height{0};
    std::string focus_mode;
    std::string lens_position;
    double fx_px{0.0};
    double fy_px{0.0};
    double cx_px{0.0};
    double cy_px{0.0};
    std::array<double, kDistortionCoefficientCount> distortion{};
};

void validate_camera_calibration(const CameraCalibration& calibration);

} // namespace onboard_autonomy::mission
