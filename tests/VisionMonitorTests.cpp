#include "TestCases.hpp"

#include "onboard_autonomy/adapters/vision/AprilTagTargetDetector.hpp"
#include "onboard_autonomy/adapters/vision/CameraGeometry.hpp"
#include "onboard_autonomy/application/AppSnapshot.hpp"
#include "onboard_autonomy/application/VisionMonitor.hpp"

#include <apriltag.h>
#include <common/image_u8.h>
#include <tagStandard41h12.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class FakeTargetDetector final
    : public onboard_autonomy::application::ports::TargetDetector {
  public:
    [[nodiscard]] onboard_autonomy::domain::TargetDetectionBatch detect(
        const onboard_autonomy::application::ports::CameraFrame& frame)
        override {
        std::vector<onboard_autonomy::domain::TargetObservation> targets;
        if (frame.sequence == 1U) {
            targets.push_back({
                .id = 7,
                .family = "fake41h12",
                .center = {.x_px = 100.0, .y_px = 80.0},
                .corners = {},
                .corrected_bits = 0,
                .decision_margin = 90.0,
                .pose = std::nullopt,
            });
        }
        return {
            .frame_sequence = frame.sequence,
            .captured_at = frame.captured_at,
            .detected_at = frame.received_at,
            .processing_time =
                std::chrono::microseconds(frame.sequence == 1U ? 2000 : 4000),
            .targets = std::move(targets),
        };
    }

    [[nodiscard]] std::string description() const override {
        return "fake detector";
    }
};

class PoseTargetDetector final
    : public onboard_autonomy::application::ports::TargetDetector {
  public:
    [[nodiscard]] onboard_autonomy::domain::TargetDetectionBatch detect(
        const onboard_autonomy::application::ports::CameraFrame& frame)
        override {
        const double right_m = frame.sequence == 1U ? 0.0 : 1.0;
        const double forward_m =
            frame.sequence == 1U ? 1.0 : (frame.sequence == 2U ? 1.2 : 0.8);
        return {
            .frame_sequence = frame.sequence,
            .captured_at = frame.captured_at,
            .detected_at = frame.received_at,
            .processing_time = std::chrono::microseconds{1000},
            .targets =
                {
                    {
                        .id = 0,
                        .family = "tagStandard41h12",
                        .center = {},
                        .corners = {},
                        .corrected_bits = 0,
                        .decision_margin = 70.0,
                        .pose =
                            onboard_autonomy::domain::TargetPose{
                                .position =
                                    {
                                        .right_m = right_m,
                                        .down_m = 0.0,
                                        .forward_m = forward_m,
                                    },
                                .rotation_tag_to_camera = {},
                                .object_space_error = 0.001,
                            },
                    },
                },
        };
    }

    [[nodiscard]] std::string description() const override {
        return "fake pose detector";
    }
};

onboard_autonomy::application::ports::CameraFrame empty_frame(
    const std::uint64_t sequence) {
    return {
        .sequence = sequence,
        .width = 320,
        .height = 240,
        .yuv420 = std::vector<std::uint8_t>(320U * 240U * 3U / 2U),
        .captured_at = std::nullopt,
        .received_at = std::chrono::system_clock::now(),
    };
}

void vision_monitor_tracks_processing_and_detections() {
    using namespace std::chrono_literals;

    FakeTargetDetector detector;
    onboard_autonomy::application::VisionMonitor monitor{detector};
    const onboard_autonomy::domain::TimePoint start{};

    monitor.process(empty_frame(1), start);
    auto snapshot = monitor.snapshot(start);
    require(snapshot.processed_frames == 1U &&
                snapshot.frames_with_targets == 1U &&
                snapshot.total_targets == 1U &&
                snapshot.latest_targets.size() == 1U &&
                snapshot.latest_targets.front().id == 7,
        "vision monitor must expose the detected target");

    monitor.process(empty_frame(2), start + 10ms);
    snapshot = monitor.snapshot(start + 25ms);
    require(snapshot.processed_frames == 2U &&
                snapshot.frames_with_targets == 1U &&
                snapshot.total_targets == 1U && snapshot.latest_targets.empty(),
        "a missing target must not look like a current detection");
    require(snapshot.latest_processing_ms.has_value() &&
                std::abs(*snapshot.latest_processing_ms - 4.0) < 0.001 &&
                snapshot.average_processing_ms.has_value() &&
                std::abs(*snapshot.average_processing_ms - 3.0) < 0.001 &&
                snapshot.maximum_processing_ms.has_value() &&
                std::abs(*snapshot.maximum_processing_ms - 4.0) < 0.001,
        "vision monitor must calculate processing latency");
    require(snapshot.last_detection_age_ms.has_value() &&
                *snapshot.last_detection_age_ms > 24.9,
        "vision monitor must retain the age of the last detection");
}

void vision_monitor_exposes_the_smoothed_confirmed_track() {
    using namespace std::chrono_literals;
    PoseTargetDetector detector;
    onboard_autonomy::application::VisionMonitor monitor{
        detector,
        {
            .required_consecutive_observations = 3,
            .loss_timeout = 500ms,
            .position_smoothing_factor = 0.5,
            .minimum_decision_margin = 20.0,
        },
    };
    const onboard_autonomy::domain::TimePoint start{};

    monitor.process(empty_frame(1), start);
    monitor.process(empty_frame(2), start + 20ms);
    const auto& current_targets = monitor.process(empty_frame(3), start + 40ms);
    const auto snapshot = monitor.snapshot(start + 40ms);

    require(
        snapshot.target_track.phase ==
                onboard_autonomy::application::TargetTrackPhase::tracking &&
            snapshot.target_track.position.has_value() &&
            std::abs(snapshot.target_track.position->right_m - 0.75) < 1.0e-9 &&
            std::abs(snapshot.target_track.position->forward_m - 0.95) < 1.0e-9,
        "vision monitor must expose the confirmed filtered track");
    require(current_targets.size() == 1U &&
                current_targets.front().pose.has_value() &&
                std::abs(current_targets.front().pose->position.right_m -
                         0.75) < 1.0e-9,
        "preview observations must use the filtered track position");
}

void real_apriltag_adapter_detects_generated_id_zero() {
    constexpr std::uint32_t frame_width = 320;
    constexpr std::uint32_t frame_height = 240;
    constexpr std::uint32_t scale = 18;

    apriltag_family_t* family = tagStandard41h12_create();
    require(family != nullptr, "test AprilTag family must be created");
    image_u8_t* tag = apriltag_to_image(family, 0);
    if (tag == nullptr) {
        tagStandard41h12_destroy(family);
        throw std::runtime_error("test AprilTag image must be created");
    }

    auto frame = empty_frame(1);
    std::fill(frame.yuv420.begin(),
        frame.yuv420.begin() +
            static_cast<std::ptrdiff_t>(frame_width * frame_height),
        static_cast<std::uint8_t>(255));
    std::fill(frame.yuv420.begin() +
                  static_cast<std::ptrdiff_t>(frame_width * frame_height),
        frame.yuv420.end(),
        static_cast<std::uint8_t>(128));

    const auto rendered_width = static_cast<std::uint32_t>(tag->width) * scale;
    const auto rendered_height =
        static_cast<std::uint32_t>(tag->height) * scale;
    const auto offset_x = (frame_width - rendered_width) / 2U;
    const auto offset_y = (frame_height - rendered_height) / 2U;
    for (std::uint32_t source_y = 0;
         source_y < static_cast<std::uint32_t>(tag->height);
         ++source_y) {
        for (std::uint32_t source_x = 0;
             source_x < static_cast<std::uint32_t>(tag->width);
             ++source_x) {
            const auto value =
                tag->buf[source_y * static_cast<std::uint32_t>(tag->stride) +
                         source_x];
            for (std::uint32_t dy = 0; dy < scale; ++dy) {
                for (std::uint32_t dx = 0; dx < scale; ++dx) {
                    const auto x = offset_x + source_x * scale + dx;
                    const auto y = offset_y + source_y * scale + dy;
                    frame.yuv420[y * frame_width + x] = value;
                }
            }
        }
    }

    image_u8_destroy(tag);
    tagStandard41h12_destroy(family);

    auto detector =
        onboard_autonomy::adapters::vision::make_apriltag_target_detector();
    const auto result = detector->detect(frame);
    require(result.targets.size() == 1U,
        "real AprilTag adapter must find one generated tag");
    const auto& target = result.targets.front();
    require(target.id == 0 && target.family == "tagStandard41h12" &&
                target.corrected_bits == 0,
        "detected tag identity must match the generated family and ID");
    require(std::abs(target.center.x_px - 159.5) < 2.0 &&
                std::abs(target.center.y_px - 119.5) < 2.0 &&
                target.decision_margin > 20.0,
        "detected center and quality must match the rendered marker");
}

onboard_autonomy::domain::CameraCalibration test_calibration() {
    return {
        .camera_model = "synthetic",
        .image_width = 320,
        .image_height = 240,
        .focus_mode = "fixed",
        .lens_position = "test",
        .fx_px = 300.0,
        .fy_px = 300.0,
        .cx_px = 159.5,
        .cy_px = 119.5,
        .distortion = {},
    };
}

void distortion_round_trip_recovers_undistorted_point() {
    auto calibration = test_calibration();
    calibration.distortion = {0.08, -0.03, 0.002, -0.001, 0.01};
    constexpr double source_x = 0.31;
    constexpr double source_y = -0.24;
    const double radius_squared = source_x * source_x + source_y * source_y;
    const double radial =
        1.0 + calibration.distortion[0] * radius_squared +
        calibration.distortion[1] * radius_squared * radius_squared +
        calibration.distortion[4] * radius_squared * radius_squared *
            radius_squared;
    const double distorted_x =
        source_x * radial +
        2.0 * calibration.distortion[2] * source_x * source_y +
        calibration.distortion[3] *
            (radius_squared + 2.0 * source_x * source_x);
    const double distorted_y =
        source_y * radial +
        calibration.distortion[2] *
            (radius_squared + 2.0 * source_y * source_y) +
        2.0 * calibration.distortion[3] * source_x * source_y;

    const auto recovered =
        onboard_autonomy::adapters::vision::undistort_image_point(
            {
                .x_px = calibration.fx_px * distorted_x + calibration.cx_px,
                .y_px = calibration.fy_px * distorted_y + calibration.cy_px,
            },
            calibration);
    require(std::abs(recovered.x_px - (calibration.fx_px * source_x +
                                          calibration.cx_px)) < 1.0e-8 &&
                std::abs(recovered.y_px - (calibration.fy_px * source_y +
                                              calibration.cy_px)) < 1.0e-8,
        "Brown-Conrady inversion must recover the source point");
}

void generated_tag_produces_metric_pose() {
    constexpr std::uint32_t frame_width = 320;
    constexpr std::uint32_t frame_height = 240;
    constexpr std::uint32_t scale = 18;
    constexpr double tag_size_m = 0.20;

    apriltag_family_t* family = tagStandard41h12_create();
    require(family != nullptr, "pose test family must be created");
    image_u8_t* tag = apriltag_to_image(family, 0);
    if (tag == nullptr) {
        tagStandard41h12_destroy(family);
        throw std::runtime_error("pose test tag must be created");
    }

    auto frame = empty_frame(2);
    std::fill(frame.yuv420.begin(),
        frame.yuv420.begin() +
            static_cast<std::ptrdiff_t>(frame_width * frame_height),
        static_cast<std::uint8_t>(255));
    const auto rendered_width = static_cast<std::uint32_t>(tag->width) * scale;
    const auto rendered_height =
        static_cast<std::uint32_t>(tag->height) * scale;
    const auto offset_x = (frame_width - rendered_width) / 2U;
    const auto offset_y = (frame_height - rendered_height) / 2U;
    for (std::uint32_t source_y = 0;
         source_y < static_cast<std::uint32_t>(tag->height);
         ++source_y) {
        for (std::uint32_t source_x = 0;
             source_x < static_cast<std::uint32_t>(tag->width);
             ++source_x) {
            const auto value =
                tag->buf[source_y * static_cast<std::uint32_t>(tag->stride) +
                         source_x];
            for (std::uint32_t dy = 0; dy < scale; ++dy) {
                for (std::uint32_t dx = 0; dx < scale; ++dx) {
                    const auto x = offset_x + source_x * scale + dx;
                    const auto y = offset_y + source_y * scale + dy;
                    frame.yuv420[y * frame_width + x] = value;
                }
            }
        }
    }
    image_u8_destroy(tag);
    tagStandard41h12_destroy(family);

    auto detector =
        onboard_autonomy::adapters::vision::make_apriltag_target_detector({
            .pose =
                onboard_autonomy::adapters::vision::AprilTagPoseConfig{
                    .calibration = test_calibration(),
                    .tag_size_m = tag_size_m,
                },
        });
    const auto result = detector->detect(frame);
    require(result.targets.size() == 1U &&
                result.targets.front().pose.has_value(),
        "calibrated detector must estimate a metric tag pose");

    const auto& target = result.targets.front();
    const auto edge_length = [](const auto& first, const auto& second) {
        return std::hypot(first.x_px - second.x_px, first.y_px - second.y_px);
    };
    const double average_edge_px =
        (edge_length(target.corners[0], target.corners[1]) +
            edge_length(target.corners[1], target.corners[2]) +
            edge_length(target.corners[2], target.corners[3]) +
            edge_length(target.corners[3], target.corners[0])) /
        4.0;
    const double expected_forward_m =
        test_calibration().fx_px * tag_size_m / average_edge_px;
    require(std::abs(target.pose->position.right_m) < 0.01 &&
                std::abs(target.pose->position.down_m) < 0.01 &&
                std::abs(target.pose->position.forward_m - expected_forward_m) <
                    expected_forward_m * 0.08 &&
                std::isfinite(target.pose->object_space_error),
        "front-facing centered tag pose must match pinhole geometry");
}

} // namespace

void run_vision_monitor_tests() {
    vision_monitor_tracks_processing_and_detections();
    vision_monitor_exposes_the_smoothed_confirmed_track();
    real_apriltag_adapter_detects_generated_id_zero();
    distortion_round_trip_recovers_undistorted_point();
    generated_tag_produces_metric_pose();
}
