#include "TestCases.hpp"

#include "onboard_autonomy/mission/cv/detection/YoloXImagePreprocessor.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void letterbox_preserves_landscape_aspect_ratio() {
    const auto geometry =
        onboard_autonomy::mission::cv::make_yolox_letterbox_geometry(1280U,
            720U);

    require(geometry.resized_width == 640U && geometry.resized_height == 360U &&
                std::abs(geometry.scale - 0.5F) < 0.0001F,
        "YOLOX letterbox must preserve a landscape frame aspect ratio");
}

void invalid_letterbox_dimensions_are_rejected() {
    try {
        static_cast<void>(
            onboard_autonomy::mission::cv::make_yolox_letterbox_geometry(0U,
                480U));
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error("zero-width YOLOX input was accepted");
}

} // namespace

void run_yolox_image_preprocessor_tests() {
    letterbox_preserves_landscape_aspect_ratio();
    invalid_letterbox_dimensions_are_rejected();
}
