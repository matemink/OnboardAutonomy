#include "onboard_autonomy/mission/cv/detection/OpenCvDnnTargetDetector.hpp"
#include "onboard_autonomy/mission/cv/detection/YoloXImagePreprocessor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifndef ONBOARD_AUTONOMY_ENABLE_OPENCV_DNN
#define ONBOARD_AUTONOMY_ENABLE_OPENCV_DNN 0
#endif

#if ONBOARD_AUTONOMY_ENABLE_OPENCV_DNN
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace onboard_autonomy::mission::cv {

#if ONBOARD_AUTONOMY_ENABLE_OPENCV_DNN
namespace {

constexpr std::int32_t kAirplaneClassId = 4;
constexpr std::int32_t kBirdClassId = 14;
constexpr std::int32_t kKiteClassId = 33;
constexpr double kConfidenceAsPercentage = 100.0;
constexpr double kBoxCenterDivisor = 2.0;
constexpr float kBoxHalfDivisor = 2.0F;

std::string class_name(const std::int32_t class_id) {
    switch (class_id) {
    case kAirplaneClassId:
        return "airplane";
    case kBirdClassId:
        return "bird";
    case kKiteClassId:
        return "kite";
    default:
        return "aerial-object";
    }
}

void validate_config(const OpenCvDnnDetectorConfig& config) {
    if (config.model_file.empty() ||
        !std::isfinite(config.confidence_threshold) ||
        config.confidence_threshold <= 0.0F ||
        config.confidence_threshold > 1.0F ||
        !std::isfinite(config.nms_threshold) || config.nms_threshold <= 0.0F ||
        config.nms_threshold > 1.0F) {
        throw std::invalid_argument(
            "invalid OpenCV DNN detector configuration");
    }
}

std::size_t expected_yuv420_size(const mission::ports::CameraFrame& frame) {
    if (frame.width == 0U || frame.height == 0U || frame.width % 2U != 0U ||
        frame.height % 2U != 0U ||
        frame.width >
            static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        frame.height >
            static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("invalid YUV420 frame dimensions");
    }
    const auto pixels = static_cast<std::uint64_t>(frame.width) * frame.height;
    const auto bytes = pixels + pixels / 2U;
    if (bytes > std::numeric_limits<std::size_t>::max()) {
        throw std::invalid_argument("YUV420 frame is too large");
    }
    return static_cast<std::size_t>(bytes);
}

::cv::Mat to_bgr(const mission::ports::CameraFrame& frame) {
    const auto expected_size = expected_yuv420_size(frame);
    if (frame.yuv420.size() != expected_size) {
        throw std::invalid_argument("unexpected YUV420 frame size");
    }
    const auto rows = static_cast<int>(frame.height + frame.height / 2U);
    const auto columns = static_cast<int>(frame.width);
    ::cv::Mat yuv(rows, columns, CV_8UC1);
    std::copy(frame.yuv420.begin(), frame.yuv420.end(), yuv.data);
    ::cv::Mat bgr;
    ::cv::cvtColor(yuv, bgr, ::cv::COLOR_YUV2BGR_I420);
    return bgr;
}

struct LetterboxedImage {
    ::cv::Mat pixels;
    float scale{};
};

LetterboxedImage letterbox(const ::cv::Mat& source) {
    constexpr auto input_size =
        static_cast<int>(YoloXDecoderConfig::kDefaultInputSize);
    constexpr std::uint8_t padding_value = 114U;
    const auto geometry =
        make_yolox_letterbox_geometry(static_cast<std::uint32_t>(source.cols),
            static_cast<std::uint32_t>(source.rows));
    const auto resized_width = static_cast<int>(geometry.resized_width);
    const auto resized_height = static_cast<int>(geometry.resized_height);

    ::cv::Mat padded(input_size,
        input_size,
        CV_8UC3,
        ::cv::Scalar{padding_value, padding_value, padding_value});
    ::cv::Mat resized;
    ::cv::resize(source,
        resized,
        ::cv::Size{resized_width, resized_height},
        0.0,
        0.0,
        ::cv::INTER_LINEAR);
    resized.copyTo(padded(::cv::Rect{0, 0, resized_width, resized_height}));
    return {.pixels = std::move(padded), .scale = geometry.scale};
}

::cv::Rect candidate_box(const YoloXCandidate& candidate,
    const float scale,
    const ::cv::Size& image_size) {
    const auto left = static_cast<int>(std::floor(
        (candidate.center_x - candidate.width / kBoxHalfDivisor) / scale));
    const auto top = static_cast<int>(std::floor(
        (candidate.center_y - candidate.height / kBoxHalfDivisor) / scale));
    const auto right = static_cast<int>(std::ceil(
        (candidate.center_x + candidate.width / kBoxHalfDivisor) / scale));
    const auto bottom = static_cast<int>(std::ceil(
        (candidate.center_y + candidate.height / kBoxHalfDivisor) / scale));
    const auto clipped_left = std::clamp(left, 0, image_size.width);
    const auto clipped_top = std::clamp(top, 0, image_size.height);
    const auto clipped_right = std::clamp(right, 0, image_size.width);
    const auto clipped_bottom = std::clamp(bottom, 0, image_size.height);
    return {clipped_left,
        clipped_top,
        std::max(0, clipped_right - clipped_left),
        std::max(0, clipped_bottom - clipped_top)};
}

mission::TargetObservation make_observation(const ::cv::Rect& box,
    const YoloXCandidate& candidate) {
    const auto left = static_cast<double>(box.x);
    const auto top = static_cast<double>(box.y);
    const auto right = static_cast<double>(box.x + box.width);
    const auto bottom = static_cast<double>(box.y + box.height);
    return {
        .id = candidate.class_id,
        .family = class_name(candidate.class_id),
        .center = {.x_px = (left + right) / kBoxCenterDivisor,
            .y_px = (top + bottom) / kBoxCenterDivisor},
        .corners = {{
            {.x_px = left, .y_px = top},
            {.x_px = right, .y_px = top},
            {.x_px = right, .y_px = bottom},
            {.x_px = left, .y_px = bottom},
        }},
        .corrected_bits = 0,
        .decision_margin =
            static_cast<double>(candidate.confidence) * kConfidenceAsPercentage,
        .pose = std::nullopt,
    };
}

std::vector<std::int32_t> detected_class_ids(
    const std::span<const YoloXCandidate> candidates) {
    std::vector<std::int32_t> result;
    result.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        result.push_back(candidate.class_id);
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

class OpenCvDnnTargetDetector final : public mission::ports::TargetDetector {
  public:
    explicit OpenCvDnnTargetDetector(const OpenCvDnnDetectorConfig& config)
        : config_(config),
          decoder_config_{
              .accepted_class_ids = config.accepted_class_ids,
              .confidence_threshold = config.confidence_threshold,
          } {
        validate_config(config_);
        try {
            network_ = ::cv::dnn::readNetFromONNX(config_.model_file.string());
        } catch (const ::cv::Exception& error) {
            throw std::runtime_error("unable to load YOLOX ONNX model: " +
                                     std::string(error.what()));
        }
        if (network_.empty()) {
            throw std::runtime_error("unable to load YOLOX ONNX model");
        }
        network_.setPreferableBackend(::cv::dnn::DNN_BACKEND_OPENCV);
        network_.setPreferableTarget(::cv::dnn::DNN_TARGET_CPU);
    }

    [[nodiscard]] mission::TargetDetectionBatch detect(
        const mission::ports::CameraFrame& frame) override {
        const auto started = std::chrono::steady_clock::now();
        const auto bgr = to_bgr(frame);
        const auto input = letterbox(bgr);
        const auto blob = ::cv::dnn::blobFromImage(input.pixels,
            1.0,
            {},
            {},
            true,
            false,
            CV_32F);
        network_.setInput(blob);
        std::vector<::cv::Mat> outputs;
        network_.forward(outputs, network_.getUnconnectedOutLayersNames());
        if (outputs.size() != 1U || outputs.front().dims != 3 ||
            outputs.front().size[0] != 1 || !outputs.front().isContinuous()) {
            throw std::runtime_error("unexpected YOLOX network output");
        }

        const auto row_count =
            static_cast<std::size_t>(outputs.front().size[1]);
        const auto column_count =
            static_cast<std::size_t>(outputs.front().size[2]);
        const auto values = std::span<const float>{
            outputs.front().ptr<float>(),
            outputs.front().total(),
        };
        const auto candidates = decode_yolox_output(values,
            row_count,
            column_count,
            decoder_config_);

        std::vector<mission::TargetObservation> observations;
        for (const auto class_id : detected_class_ids(candidates)) {
            std::vector<::cv::Rect> boxes;
            std::vector<float> scores;
            std::vector<const YoloXCandidate*> selected_candidates;
            for (const auto& candidate : candidates) {
                if (candidate.class_id != class_id) {
                    continue;
                }
                const auto box =
                    candidate_box(candidate, input.scale, bgr.size());
                if (box.empty()) {
                    continue;
                }
                boxes.push_back(box);
                scores.push_back(candidate.confidence);
                selected_candidates.push_back(&candidate);
            }
            std::vector<int> retained;
            ::cv::dnn::NMSBoxes(boxes,
                scores,
                config_.confidence_threshold,
                config_.nms_threshold,
                retained);
            for (const auto index : retained) {
                const auto selected_index = static_cast<std::size_t>(index);
                observations.push_back(
                    make_observation(boxes.at(selected_index),
                        *selected_candidates.at(selected_index)));
            }
        }

        const auto finished = std::chrono::steady_clock::now();
        return {
            .frame_sequence = frame.sequence,
            .captured_at = frame.captured_at,
            .detected_at = std::chrono::system_clock::now(),
            .processing_time =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    finished - started),
            .targets = std::move(observations),
        };
    }

    [[nodiscard]] std::string description() const override {
        return "OpenCV DNN YOLOX (" + config_.model_file.filename().string() +
               ')';
    }

  private:
    OpenCvDnnDetectorConfig config_;
    YoloXDecoderConfig decoder_config_;
    ::cv::dnn::Net network_;
};

} // namespace
#endif

std::unique_ptr<mission::ports::TargetDetector> make_opencv_dnn_target_detector(
    const OpenCvDnnDetectorConfig& config) {
#if ONBOARD_AUTONOMY_ENABLE_OPENCV_DNN
    return std::make_unique<OpenCvDnnTargetDetector>(config);
#else
    static_cast<void>(config);
    throw std::runtime_error("OpenCV DNN support is disabled in this build");
#endif
}

} // namespace onboard_autonomy::mission::cv
