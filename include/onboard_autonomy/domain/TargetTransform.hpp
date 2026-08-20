#pragma once

#include "onboard_autonomy/domain/TargetObservation.hpp"

#include <array>

namespace onboard_autonomy::domain {

struct BodyFramePosition {
    double forward_m{0.0};
    double right_m{0.0};
    double down_m{0.0};
};

struct CameraExtrinsics {
    // Row-major rotation from [camera right, down, forward] to body FRD.
    RotationMatrix rotation_camera_to_body{};
    BodyFramePosition camera_origin_in_body;
};

void validate_camera_extrinsics(const CameraExtrinsics& extrinsics);

[[nodiscard]] BodyFramePosition camera_to_body_frd(
    const CameraFramePosition& camera_position,
    const CameraExtrinsics& extrinsics);

} // namespace onboard_autonomy::domain
