#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace onboard_autonomy::mission::cv {

struct YoloXCandidate {
    float center_x{};
    float center_y{};
    float width{};
    float height{};
    float confidence{};
    std::int32_t class_id{};
};

struct YoloXDecoderConfig {
    static constexpr std::size_t kCocoClassCount = 80U;
    static constexpr std::uint32_t kDefaultInputSize = 640U;
    static constexpr std::array<std::uint32_t, 3> kDefaultStrides{
        8U,
        16U,
        32U,
    };
    static constexpr std::array<std::int32_t, 3> kAerialClassIds{
        4,
        14,
        33,
    };
    static constexpr float kDefaultConfidenceThreshold = 0.20F;

    std::size_t class_count{kCocoClassCount};
    std::uint32_t input_size{kDefaultInputSize};
    std::vector<std::uint32_t> strides{
        kDefaultStrides.begin(),
        kDefaultStrides.end(),
    };
    std::vector<std::int32_t> accepted_class_ids{
        kAerialClassIds.begin(),
        kAerialClassIds.end(),
    };
    float confidence_threshold{kDefaultConfidenceThreshold};
};

[[nodiscard]] std::vector<YoloXCandidate> decode_yolox_output(
    std::span<const float> output,
    std::size_t row_count,
    std::size_t column_count,
    const YoloXDecoderConfig& config = {});

} // namespace onboard_autonomy::mission::cv
