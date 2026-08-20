#include "onboard_autonomy/domain/TargetTransform.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace onboard_autonomy::domain {
namespace {

constexpr double kRotationTolerance = 1.0e-6;

double row_dot(const RotationMatrix& rotation,
    const std::size_t first,
    const std::size_t second) {
    double result = 0.0;
    for (std::size_t column = 0; column < kRotationMatrixDimension; ++column) {
        result += rotation[first * kRotationMatrixDimension + column] *
                  rotation[second * kRotationMatrixDimension + column];
    }
    return result;
}

double determinant(const RotationMatrix& rotation) {
    const auto at = [&rotation](const std::size_t row,
                        const std::size_t column) {
        return rotation[row * kRotationMatrixDimension + column];
    };
    return at(0, 0) * (at(1, 1) * at(2, 2) - at(1, 2) * at(2, 1)) -
           at(0, 1) * (at(1, 0) * at(2, 2) - at(1, 2) * at(2, 0)) +
           at(0, 2) * (at(1, 0) * at(2, 1) - at(1, 1) * at(2, 0));
}

} // namespace

void validate_camera_extrinsics(const CameraExtrinsics& extrinsics) {
    const auto finite = [](const double value) { return std::isfinite(value); };
    if (!std::ranges::all_of(extrinsics.rotation_camera_to_body, finite) ||
        !finite(extrinsics.camera_origin_in_body.forward_m) ||
        !finite(extrinsics.camera_origin_in_body.right_m) ||
        !finite(extrinsics.camera_origin_in_body.down_m)) {
        throw std::invalid_argument(
            "camera extrinsics must contain finite values");
    }

    for (std::size_t row = 0; row < kRotationMatrixDimension; ++row) {
        if (std::abs(row_dot(extrinsics.rotation_camera_to_body, row, row) -
                     1.0) > kRotationTolerance) {
            throw std::invalid_argument(
                "camera rotation rows must have unit length");
        }
        for (std::size_t other = row + 1U; other < kRotationMatrixDimension;
             ++other) {
            if (std::abs(
                    row_dot(extrinsics.rotation_camera_to_body, row, other)) >
                kRotationTolerance) {
                throw std::invalid_argument(
                    "camera rotation rows must be orthogonal");
            }
        }
    }

    if (std::abs(determinant(extrinsics.rotation_camera_to_body) - 1.0) >
        kRotationTolerance) {
        throw std::invalid_argument(
            "camera rotation must be a right-handed rigid transform");
    }
}

BodyFramePosition camera_to_body_frd(const CameraFramePosition& camera_position,
    const CameraExtrinsics& extrinsics) {
    validate_camera_extrinsics(extrinsics);
    if (!std::isfinite(camera_position.right_m) ||
        !std::isfinite(camera_position.down_m) ||
        !std::isfinite(camera_position.forward_m)) {
        throw std::invalid_argument("camera target position must be finite");
    }

    const std::array<double, kRotationMatrixDimension> camera{
        camera_position.right_m,
        camera_position.down_m,
        camera_position.forward_m,
    };
    std::array<double, kRotationMatrixDimension> body{};
    for (std::size_t row = 0; row < kRotationMatrixDimension; ++row) {
        for (std::size_t column = 0; column < kRotationMatrixDimension;
             ++column) {
            body[row] +=
                extrinsics
                    .rotation_camera_to_body[row * kRotationMatrixDimension +
                                             column] *
                camera[column];
        }
    }

    return {
        .forward_m = body[0] + extrinsics.camera_origin_in_body.forward_m,
        .right_m = body[1] + extrinsics.camera_origin_in_body.right_m,
        .down_m = body[2] + extrinsics.camera_origin_in_body.down_m,
    };
}

} // namespace onboard_autonomy::domain
