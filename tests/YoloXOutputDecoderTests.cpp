#include "TestCases.hpp"

#include "onboard_autonomy/mission/cv/detection/YoloXOutputDecoder.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kClassScoresOffset = 5U;
constexpr std::int32_t kKiteClassId = 33;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

onboard_autonomy::mission::cv::YoloXDecoderConfig single_cell_config() {
    return {
        .class_count = 80U,
        .input_size = 8U,
        .strides = {8U},
        .accepted_class_ids = {},
        .confidence_threshold = 0.20F,
    };
}

void detected_class_is_decoded_from_raw_yolox_values() {
    const auto config = single_cell_config();
    std::vector<float> output(kClassScoresOffset + config.class_count, 0.0F);
    output[0U] = 0.5F;
    output[1U] = 0.25F;
    output[2U] = std::log(2.0F);
    output[3U] = std::log(1.5F);
    output[4U] = 0.8F;
    output[kClassScoresOffset + static_cast<std::size_t>(kKiteClassId)] = 0.75F;

    const auto candidates =
        onboard_autonomy::mission::cv::decode_yolox_output(output,
            1U,
            output.size(),
            config);

    require(candidates.size() == 1U, "detected COCO class must be decoded");
    const auto& candidate = candidates.front();
    require(candidate.class_id == kKiteClassId &&
                std::abs(candidate.confidence - 0.6F) < 0.0001F &&
                std::abs(candidate.center_x - 4.0F) < 0.0001F &&
                std::abs(candidate.center_y - 2.0F) < 0.0001F &&
                std::abs(candidate.width - 16.0F) < 0.0001F &&
                std::abs(candidate.height - 12.0F) < 0.0001F,
        "YOLOX grid and stride transform must match the model contract");
}

void confidence_and_shape_are_enforced() {
    const auto config = single_cell_config();
    std::vector<float> output(kClassScoresOffset + config.class_count, 0.0F);
    output[2U] = 0.0F;
    output[3U] = 0.0F;
    output[4U] = 0.5F;
    output[kClassScoresOffset + static_cast<std::size_t>(kKiteClassId)] = 0.2F;

    require(onboard_autonomy::mission::cv::decode_yolox_output(output,
                1U,
                output.size(),
                config)
                .empty(),
        "candidate below the configured confidence must be ignored");

    try {
        static_cast<void>(
            onboard_autonomy::mission::cv::decode_yolox_output(output,
                2U,
                output.size(),
                config));
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error("unexpected YOLOX tensor shape was accepted");
}

void confidence_must_be_strictly_above_the_threshold() {
    auto config = single_cell_config();
    config.confidence_threshold = 0.51F;
    std::vector<float> output(kClassScoresOffset + config.class_count, 0.0F);
    output[2U] = 0.0F;
    output[3U] = 0.0F;
    output[4U] = 1.0F;
    output[kClassScoresOffset + static_cast<std::size_t>(kKiteClassId)] =
        config.confidence_threshold;

    require(onboard_autonomy::mission::cv::decode_yolox_output(output,
                1U,
                output.size(),
                config)
                .empty(),
        "candidate at the configured confidence must be ignored");
}

void dominant_class_is_decoded_without_a_category_filter() {
    constexpr std::int32_t person_class_id = 0;
    const auto config = single_cell_config();
    std::vector<float> output(kClassScoresOffset + config.class_count, 0.0F);
    output[2U] = 0.0F;
    output[3U] = 0.0F;
    output[4U] = 1.0F;
    output[kClassScoresOffset + static_cast<std::size_t>(person_class_id)] =
        0.90F;
    output[kClassScoresOffset + static_cast<std::size_t>(kKiteClassId)] = 0.80F;

    const auto candidates =
        onboard_autonomy::mission::cv::decode_yolox_output(output,
            1U,
            output.size(),
            config);
    require(candidates.size() == 1U &&
                candidates.front().class_id == person_class_id,
        "the detector's dominant class must pass without a category filter");
}

void explicit_category_filter_remains_available() {
    constexpr std::int32_t person_class_id = 0;
    auto config = single_cell_config();
    config.accepted_class_ids = {kKiteClassId};
    std::vector<float> output(kClassScoresOffset + config.class_count, 0.0F);
    output[4U] = 1.0F;
    output[kClassScoresOffset + static_cast<std::size_t>(person_class_id)] =
        0.90F;

    require(onboard_autonomy::mission::cv::decode_yolox_output(output,
                1U,
                output.size(),
                config)
                .empty(),
        "an explicit category filter must still reject other classes");
}

void category_filter_selects_the_best_allowed_class() {
    constexpr std::int32_t person_class_id = 0;
    auto config = single_cell_config();
    config.accepted_class_ids = {kKiteClassId};
    std::vector<float> output(kClassScoresOffset + config.class_count, 0.0F);
    output[2U] = 0.0F;
    output[3U] = 0.0F;
    output[4U] = 1.0F;
    output[kClassScoresOffset + static_cast<std::size_t>(person_class_id)] =
        0.95F;
    output[kClassScoresOffset + static_cast<std::size_t>(kKiteClassId)] = 0.80F;

    const auto candidates =
        onboard_autonomy::mission::cv::decode_yolox_output(output,
            1U,
            output.size(),
            config);
    require(candidates.size() == 1U &&
                candidates.front().class_id == kKiteClassId,
        "a stronger rejected class must not hide an accepted class");
}

} // namespace

void run_yolox_output_decoder_tests() {
    detected_class_is_decoded_from_raw_yolox_values();
    confidence_and_shape_are_enforced();
    confidence_must_be_strictly_above_the_threshold();
    dominant_class_is_decoded_without_a_category_filter();
    explicit_category_filter_remains_available();
    category_filter_selects_the_best_allowed_class();
}
