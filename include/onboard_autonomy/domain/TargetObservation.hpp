#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::domain {

inline constexpr std::size_t kRotationMatrixDimension = 3;
inline constexpr std::size_t kRotationMatrixElementCount =
    kRotationMatrixDimension * kRotationMatrixDimension;
using RotationMatrix = std::array<double, kRotationMatrixElementCount>;

struct ImagePoint {
    double x_px{0.0};
    double y_px{0.0};
};

struct CameraFramePosition {
    double right_m{0.0};
    double down_m{0.0};
    double forward_m{0.0};
};

struct TargetPose {
    CameraFramePosition position;
    RotationMatrix rotation_tag_to_camera{};
    double object_space_error{0.0};
};

struct TargetObservation {
    std::int32_t id{0};
    std::string family;
    ImagePoint center;
    std::array<ImagePoint, 4> corners;
    std::int32_t corrected_bits{0};
    double decision_margin{0.0};
    std::optional<TargetPose> pose;
};

struct TargetDetectionBatch {
    std::uint64_t frame_sequence{0};
    std::optional<std::chrono::system_clock::time_point> captured_at;
    std::chrono::system_clock::time_point detected_at;
    std::chrono::microseconds processing_time{};
    std::vector<TargetObservation> targets;
};

} // namespace onboard_autonomy::domain
