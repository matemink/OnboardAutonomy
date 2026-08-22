#pragma once

#include "onboard_autonomy/mission/cv/detection/YoloXOutputDecoder.hpp"

#include <cstdint>

namespace onboard_autonomy::mission::cv {

struct YoloXLetterboxGeometry {
    std::uint32_t resized_width{};
    std::uint32_t resized_height{};
    float scale{};
};

[[nodiscard]] YoloXLetterboxGeometry make_yolox_letterbox_geometry(
    std::uint32_t source_width,
    std::uint32_t source_height,
    std::uint32_t input_size = YoloXDecoderConfig::kDefaultInputSize);

} // namespace onboard_autonomy::mission::cv
