#include "onboard_autonomy/mission/cv/detection/YoloXOutputDecoder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace onboard_autonomy::mission::cv {
namespace {

constexpr std::size_t kBoxValueCount = 4U;
constexpr std::size_t kObjectnessIndex = 4U;
constexpr std::size_t kClassScoresOffset = 5U;

struct GridCell {
    float x{};
    float y{};
    float stride{};
};

void validate_config(const YoloXDecoderConfig& config) {
    if (config.class_count == 0U || config.input_size == 0U ||
        config.strides.empty() || !std::isfinite(config.confidence_threshold) ||
        config.confidence_threshold < 0.0F ||
        config.confidence_threshold > 1.0F) {
        throw std::invalid_argument("invalid YOLOX decoder configuration");
    }
    for (const auto stride : config.strides) {
        if (stride == 0U || config.input_size % stride != 0U) {
            throw std::invalid_argument(
                "YOLOX stride must divide the input size");
        }
    }
    for (const auto class_id : config.accepted_class_ids) {
        if (class_id < 0 ||
            static_cast<std::size_t>(class_id) >= config.class_count) {
            throw std::invalid_argument("YOLOX class id is out of range");
        }
    }
}

std::vector<GridCell> make_grid(const YoloXDecoderConfig& config) {
    std::vector<GridCell> result;
    for (const auto stride : config.strides) {
        const auto side = config.input_size / stride;
        result.reserve(result.size() + static_cast<std::size_t>(side) * side);
        for (std::uint32_t y = 0U; y < side; ++y) {
            for (std::uint32_t x = 0U; x < side; ++x) {
                result.push_back({
                    .x = static_cast<float>(x),
                    .y = static_cast<float>(y),
                    .stride = static_cast<float>(stride),
                });
            }
        }
    }
    return result;
}

std::pair<std::int32_t, float> highest_scoring_class(
    const std::span<const float> row,
    const YoloXDecoderConfig& config) {
    std::int32_t selected_class = -1;
    float selected_score = 0.0F;
    for (std::size_t class_index = 0U; class_index < config.class_count;
         ++class_index) {
        const auto score = row[kClassScoresOffset + class_index];
        if (std::isfinite(score) && score > selected_score) {
            selected_score = score;
            selected_class = static_cast<std::int32_t>(class_index);
        }
    }
    return {selected_class, selected_score};
}

bool accepted_class(const std::int32_t class_id,
    const YoloXDecoderConfig& config) {
    return config.accepted_class_ids.empty() ||
           std::find(config.accepted_class_ids.begin(),
               config.accepted_class_ids.end(),
               class_id) != config.accepted_class_ids.end();
}

} // namespace

std::vector<YoloXCandidate> decode_yolox_output(
    const std::span<const float> output,
    const std::size_t row_count,
    const std::size_t column_count,
    const YoloXDecoderConfig& config) {
    validate_config(config);
    const auto grid = make_grid(config);
    const auto expected_columns = kClassScoresOffset + config.class_count;
    if (row_count != grid.size() || column_count != expected_columns ||
        output.size() != row_count * column_count) {
        throw std::invalid_argument("unexpected YOLOX output shape");
    }

    std::vector<YoloXCandidate> result;
    for (std::size_t row_index = 0U; row_index < row_count; ++row_index) {
        const auto row = output.subspan(row_index * column_count, column_count);
        const auto objectness = row[kObjectnessIndex];
        const auto [class_id, class_score] = highest_scoring_class(row, config);
        const auto confidence = objectness * class_score;
        if (class_id < 0 || !accepted_class(class_id, config) ||
            !std::isfinite(objectness) || !std::isfinite(confidence) ||
            confidence < config.confidence_threshold) {
            continue;
        }

        const auto& cell = grid[row_index];
        const auto center_x = (row[0U] + cell.x) * cell.stride;
        const auto center_y = (row[1U] + cell.y) * cell.stride;
        const auto width = std::exp(row[2U]) * cell.stride;
        const auto height = std::exp(row[3U]) * cell.stride;
        if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
            !std::isfinite(width) || !std::isfinite(height) || width <= 0.0F ||
            height <= 0.0F) {
            continue;
        }
        result.push_back({
            .center_x = center_x,
            .center_y = center_y,
            .width = width,
            .height = height,
            .confidence = confidence,
            .class_id = class_id,
        });
    }
    return result;
}

} // namespace onboard_autonomy::mission::cv
