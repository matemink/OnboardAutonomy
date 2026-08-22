#include "onboard_autonomy/mission/cv/detection/YoloXImagePreprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace onboard_autonomy::mission::cv {

YoloXLetterboxGeometry make_yolox_letterbox_geometry(
    const std::uint32_t source_width,
    const std::uint32_t source_height,
    const std::uint32_t input_size) {
    if (source_width == 0U || source_height == 0U || input_size == 0U) {
        throw std::invalid_argument(
            "YOLOX letterbox dimensions must be non-zero");
    }

    const auto horizontal_scale =
        static_cast<float>(input_size) / static_cast<float>(source_width);
    const auto vertical_scale =
        static_cast<float>(input_size) / static_cast<float>(source_height);
    const auto scale = std::min(horizontal_scale, vertical_scale);
    return {
        .resized_width = static_cast<std::uint32_t>(
            std::round(static_cast<float>(source_width) * scale)),
        .resized_height = static_cast<std::uint32_t>(
            std::round(static_cast<float>(source_height) * scale)),
        .scale = scale,
    };
}

} // namespace onboard_autonomy::mission::cv
