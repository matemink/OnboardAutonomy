#include "onboard_autonomy/adapters/vision/AprilTagTargetDetector.hpp"
#include "onboard_autonomy/adapters/vision/CameraGeometry.hpp"

#include <apriltag.h>
#include <apriltag_pose.h>
#include <common/homography.h>
#include <common/image_types.h>
#include <common/matd.h>
#include <common/zarray.h>
#include <tagStandard41h12.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace onboard_autonomy::adapters::vision {
namespace {

class DetectionArray {
public:
    explicit DetectionArray(zarray_t* detections)
        : detections_(detections) {}

    ~DetectionArray() {
        if (detections_ != nullptr) {
            apriltag_detections_destroy(detections_);
        }
    }

    DetectionArray(const DetectionArray&) = delete;
    DetectionArray& operator=(const DetectionArray&) = delete;

    [[nodiscard]] zarray_t* get() const {
        return detections_;
    }

private:
    zarray_t* detections_;
};

class Matrix {
public:
    explicit Matrix(matd_t* matrix)
        : matrix_(matrix) {}

    ~Matrix() {
        matd_destroy(matrix_);
    }

    Matrix(const Matrix&) = delete;
    Matrix& operator=(const Matrix&) = delete;

    [[nodiscard]] matd_t* get() const {
        return matrix_;
    }

private:
    matd_t* matrix_;
};

class Correspondences {
public:
    Correspondences()
        : values_(zarray_create(sizeof(float[4]))) {
        if (values_ == nullptr) {
            throw std::runtime_error(
                "unable to allocate AprilTag correspondences"
            );
        }
    }

    ~Correspondences() {
        zarray_destroy(values_);
    }

    Correspondences(const Correspondences&) = delete;
    Correspondences& operator=(const Correspondences&) = delete;

    void add(
        const float tag_x,
        const float tag_y,
        const domain::ImagePoint& image
    ) {
        const float correspondence[4]{
            tag_x,
            tag_y,
            static_cast<float>(image.x_px),
            static_cast<float>(image.y_px),
        };
        zarray_add(values_, correspondence);
    }

    [[nodiscard]] zarray_t* get() const {
        return values_;
    }

private:
    zarray_t* values_;
};

std::optional<domain::TargetPose> estimate_pose(
    const apriltag_detection_t& detection,
    const AprilTagPoseConfig& config
) {
    constexpr std::array<std::array<float, 2>, 4> tag_corners{{
        {{-1.0F, 1.0F}},
        {{1.0F, 1.0F}},
        {{1.0F, -1.0F}},
        {{-1.0F, -1.0F}},
    }};

    apriltag_detection_t corrected = detection;
    Correspondences correspondences;
    for (std::size_t index = 0; index < tag_corners.size();
         ++index) {
        const auto point = undistort_image_point(
            {
                .x_px = detection.p[index][0],
                .y_px = detection.p[index][1],
            },
            config.calibration
        );
        corrected.p[index][0] = point.x_px;
        corrected.p[index][1] = point.y_px;
        correspondences.add(
            tag_corners[index][0],
            tag_corners[index][1],
            point
        );
    }
    const auto center = undistort_image_point(
        {
            .x_px = detection.c[0],
            .y_px = detection.c[1],
        },
        config.calibration
    );
    corrected.c[0] = center.x_px;
    corrected.c[1] = center.y_px;

    Matrix homography{
        homography_compute(
            correspondences.get(),
            HOMOGRAPHY_COMPUTE_FLAG_SVD
        )
    };
    if (homography.get() == nullptr) {
        return std::nullopt;
    }
    corrected.H = homography.get();

    apriltag_detection_info_t info{
        .det = &corrected,
        .tagsize = config.tag_size_m,
        .fx = config.calibration.fx_px,
        .fy = config.calibration.fy_px,
        .cx = config.calibration.cx_px,
        .cy = config.calibration.cy_px,
    };
    apriltag_pose_t raw_pose{};
    const double error = estimate_tag_pose(&info, &raw_pose);
    Matrix rotation{raw_pose.R};
    Matrix translation{raw_pose.t};
    if (rotation.get() == nullptr || translation.get() == nullptr ||
        !std::isfinite(error)) {
        return std::nullopt;
    }

    domain::TargetPose pose{
        .position =
            {
                .right_m = MATD_EL(translation.get(), 0, 0),
                .down_m = MATD_EL(translation.get(), 1, 0),
                .forward_m = MATD_EL(translation.get(), 2, 0),
            },
        .rotation_tag_to_camera = {},
        .object_space_error = error,
    };
    for (std::size_t row = 0; row < 3U; ++row) {
        for (std::size_t column = 0; column < 3U; ++column) {
            pose.rotation_tag_to_camera[row * 3U + column] =
                MATD_EL(
                    rotation.get(),
                    static_cast<unsigned int>(row),
                    static_cast<unsigned int>(column)
                );
        }
    }

    const bool finite_position =
        std::isfinite(pose.position.right_m) &&
        std::isfinite(pose.position.down_m) &&
        std::isfinite(pose.position.forward_m);
    const bool finite_rotation = std::ranges::all_of(
        pose.rotation_tag_to_camera,
        [](const double value) {
            return std::isfinite(value);
        }
    );
    if (!finite_position || !finite_rotation ||
        pose.position.forward_m <= 0.0) {
        return std::nullopt;
    }
    return pose;
}

class AprilTagTargetDetector final
    : public application::ports::TargetDetector {
public:
    explicit AprilTagTargetDetector(
        const AprilTagDetectorConfig& config
    )
        : pose_config_(config.pose) {
        if (config.worker_threads == 0U ||
            config.worker_threads >
                static_cast<std::uint32_t>(
                    std::numeric_limits<int>::max()
                ) ||
            config.quad_decimate < 1.0 ||
            config.corrected_bits < 0 ||
            config.corrected_bits > 2) {
            throw std::invalid_argument(
                "invalid AprilTag detector configuration"
            );
        }
        if (pose_config_.has_value()) {
            domain::validate_camera_calibration(
                pose_config_->calibration
            );
            if (!std::isfinite(pose_config_->tag_size_m) ||
                pose_config_->tag_size_m <= 0.0) {
                throw std::invalid_argument(
                    "AprilTag size must be positive and finite"
                );
            }
        }

        family_ = tagStandard41h12_create();
        detector_ = apriltag_detector_create();
        if (family_ == nullptr || detector_ == nullptr) {
            if (detector_ != nullptr) {
                apriltag_detector_destroy(detector_);
            }
            if (family_ != nullptr) {
                tagStandard41h12_destroy(family_);
            }
            throw std::runtime_error(
                "unable to create AprilTag detector"
            );
        }

        detector_->nthreads =
            static_cast<int>(config.worker_threads);
        detector_->quad_decimate =
            static_cast<float>(config.quad_decimate);
        detector_->quad_sigma = 0.0F;
        detector_->refine_edges = config.refine_edges;
        detector_->debug = false;
        apriltag_detector_add_family_bits(
            detector_,
            family_,
            config.corrected_bits
        );
    }

    ~AprilTagTargetDetector() override {
        if (detector_ != nullptr) {
            apriltag_detector_destroy(detector_);
        }
        if (family_ != nullptr) {
            tagStandard41h12_destroy(family_);
        }
    }

    [[nodiscard]] domain::TargetDetectionBatch detect(
        const application::ports::CameraFrame& frame
    ) override {
        const std::uint64_t luma_bytes =
            static_cast<std::uint64_t>(frame.width) *
            static_cast<std::uint64_t>(frame.height);
        if (frame.width == 0U || frame.height == 0U ||
            luma_bytes > frame.yuv420.size() ||
            frame.width >
                static_cast<std::uint32_t>(
                    std::numeric_limits<std::int32_t>::max()
                ) ||
            frame.height >
                static_cast<std::uint32_t>(
                    std::numeric_limits<std::int32_t>::max()
                )) {
            throw std::invalid_argument(
                "AprilTag detector requires a complete Y plane"
            );
        }
        if (pose_config_.has_value() &&
            (frame.width !=
                 pose_config_->calibration.image_width ||
             frame.height !=
                 pose_config_->calibration.image_height)) {
            throw std::invalid_argument(
                "camera frame does not match calibration resolution"
            );
        }

        image_u8_t image{
            .width = static_cast<std::int32_t>(frame.width),
            .height = static_cast<std::int32_t>(frame.height),
            .stride = static_cast<std::int32_t>(frame.width),
            // AprilTag's C API is mutable, but detection treats the
            // source image as input when quad_sigma is zero.
            .buf = const_cast<std::uint8_t*>(
                frame.yuv420.data()
            ),
        };

        const auto processing_started =
            std::chrono::steady_clock::now();
        DetectionArray detections{
            apriltag_detector_detect(detector_, &image)
        };
        const auto detected_at =
            std::chrono::system_clock::now();
        const auto processing_finished =
            std::chrono::steady_clock::now();
        if (detections.get() == nullptr) {
            throw std::runtime_error(
                "AprilTag detector returned no result array"
            );
        }

        std::vector<domain::TargetObservation> targets;
        const int count = zarray_size(detections.get());
        targets.reserve(static_cast<std::size_t>(count));
        for (int index = 0; index < count; ++index) {
            apriltag_detection_t* detection = nullptr;
            zarray_get(
                detections.get(),
                index,
                static_cast<void*>(&detection)
            );
            if (detection == nullptr) {
                continue;
            }

            std::array<domain::ImagePoint, 4> corners{};
            for (std::size_t corner = 0;
                 corner < corners.size();
                 ++corner) {
                corners[corner] = {
                    .x_px = detection->p[corner][0],
                    .y_px = detection->p[corner][1],
                };
            }
            targets.push_back(
                {
                    .id = detection->id,
                    .family =
                        detection->family != nullptr &&
                                detection->family->name != nullptr
                            ? detection->family->name
                            : "tagStandard41h12",
                    .center =
                        {
                            .x_px = detection->c[0],
                            .y_px = detection->c[1],
                        },
                    .corners = corners,
                    .corrected_bits = detection->hamming,
                    .decision_margin =
                        static_cast<double>(
                            detection->decision_margin
                        ),
                    .pose = pose_config_.has_value()
                        ? estimate_pose(*detection, *pose_config_)
                        : std::nullopt,
                }
            );
        }

        return {
            .frame_sequence = frame.sequence,
            .captured_at = frame.captured_at,
            .detected_at = detected_at,
            .processing_time =
                std::chrono::duration_cast<
                    std::chrono::microseconds
                >(processing_finished - processing_started),
            .targets = std::move(targets),
        };
    }

    [[nodiscard]] std::string description() const override {
        return "AprilTag 3 / tagStandard41h12";
    }

private:
    apriltag_family_t* family_{nullptr};
    apriltag_detector_t* detector_{nullptr};
    std::optional<AprilTagPoseConfig> pose_config_;
};

}  // namespace

std::unique_ptr<application::ports::TargetDetector>
make_apriltag_target_detector(const AprilTagDetectorConfig& config) {
    return std::make_unique<AprilTagTargetDetector>(config);
}

}  // namespace onboard_autonomy::adapters::vision
