#pragma once

#include "onboard_autonomy/application/ports/TargetDetector.hpp"
#include "onboard_autonomy/domain/CameraCalibration.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace onboard_autonomy::adapters::vision {

struct AprilTagPoseConfig {
    domain::CameraCalibration calibration;
    double tag_size_m{0.0};
};

struct AprilTagDetectorConfig {
    std::uint32_t worker_threads{2};
    double quad_decimate{1.0};
    bool refine_edges{true};
    std::int32_t corrected_bits{2};
    std::optional<AprilTagPoseConfig> pose;
};

[[nodiscard]] std::unique_ptr<
    application::ports::TargetDetector
> make_apriltag_target_detector(
    const AprilTagDetectorConfig& config = {}
);

}  // namespace onboard_autonomy::adapters::vision
