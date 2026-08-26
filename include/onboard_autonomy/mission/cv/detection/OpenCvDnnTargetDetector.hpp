#pragma once

#include "onboard_autonomy/mission/cv/detection/TargetDetector.hpp"
#include "onboard_autonomy/mission/cv/detection/YoloXOutputDecoder.hpp"

#include <filesystem>
#include <memory>
#include <vector>

namespace onboard_autonomy::mission::cv {

struct OpenCvDnnDetectorConfig {
    static constexpr float kDefaultNmsThreshold = 0.50F;

    std::filesystem::path model_file;
    std::vector<std::int32_t> accepted_class_ids;
    float confidence_threshold{YoloXDecoderConfig::kDefaultConfidenceThreshold};
    float nms_threshold{kDefaultNmsThreshold};
};

[[nodiscard]] std::unique_ptr<mission::ports::TargetDetector>
make_opencv_dnn_target_detector(const OpenCvDnnDetectorConfig& config);

} // namespace onboard_autonomy::mission::cv
