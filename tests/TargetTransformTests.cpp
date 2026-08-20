#include "TestCases.hpp"

#include "onboard_autonomy/mission/cv/extrinsics/TargetTransform.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

onboard_autonomy::mission::CameraExtrinsics downward_camera() {
    return {
        .rotation_camera_to_body =
            {
                0.0,
                -1.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            },
        .camera_origin_in_body =
            {
                .forward_m = 0.0,
                .right_m = 0.0,
                .down_m = 0.16,
            },
    };
}

void downward_camera_maps_optical_axes_to_body_frd() {
    const auto body = onboard_autonomy::mission::camera_to_body_frd(
        {
            .right_m = 0.20,
            .down_m = -0.40,
            .forward_m = 1.50,
        },
        downward_camera());

    require(std::abs(body.forward_m - 0.40) < 1.0e-12 &&
                std::abs(body.right_m - 0.20) < 1.0e-12 &&
                std::abs(body.down_m - 1.66) < 1.0e-12,
        "downward camera must map optical right/down/forward to body FRD");
}

void invalid_rotation_is_rejected() {
    auto extrinsics = downward_camera();
    extrinsics.rotation_camera_to_body[0] = 2.0;

    bool rejected = false;
    try {
        onboard_autonomy::mission::validate_camera_extrinsics(extrinsics);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "non-rigid camera rotations must be rejected");
}

} // namespace

void run_target_transform_tests() {
    downward_camera_maps_optical_axes_to_body_frd();
    invalid_rotation_is_rejected();
}
