#pragma once

#include "onboard_autonomy/mission/cv/detection/TargetDetector.hpp"
#include "onboard_autonomy/mission/cv/calibration/CameraCalibration.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace onboard_autonomy::mission::cv {

struct AprilTagPoseConfig {
    mission::CameraCalibration calibration;
    double tag_size_m{0.0};
};

struct AprilTagDetectorConfig {
    std::uint32_t worker_threads{2};
    double quad_decimate{1.0};
    bool refine_edges{true};
    std::int32_t corrected_bits{2};
    std::optional<AprilTagPoseConfig> pose;
};

[[nodiscard]] std::unique_ptr<mission::ports::TargetDetector>
make_apriltag_target_detector(const AprilTagDetectorConfig& config = {});

} // namespace onboard_autonomy::mission::cv
