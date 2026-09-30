#include "TestCases.hpp"

#include "onboard_autonomy/mission/cv/detection/CameraGeometry.hpp"
#include "onboard_autonomy/mission/AppSnapshot.hpp"
#include "onboard_autonomy/mission/cv/VisionMonitor.hpp"


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
    : public onboard_autonomy::mission::ports::TargetDetector {
  public:
    [[nodiscard]] onboard_autonomy::mission::TargetDetectionBatch detect(
        const onboard_autonomy::mission::ports::CameraFrame& frame) override {
        std::vector<onboard_autonomy::mission::TargetObservation> targets;
        if (frame.sequence == 1U) {
            targets.push_back({
                .id = 7,
                .family = "synthetic-object",
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
    : public onboard_autonomy::mission::ports::TargetDetector {
  public:
    [[nodiscard]] onboard_autonomy::mission::TargetDetectionBatch detect(
        const onboard_autonomy::mission::ports::CameraFrame& frame) override {
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
                        .family = "synthetic-pose",
                        .center = {},
                        .corners = {},
                        .corrected_bits = 0,
                        .decision_margin = 70.0,
                        .pose =
                            onboard_autonomy::mission::TargetPose{
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

onboard_autonomy::mission::ports::CameraFrame empty_frame(
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
    onboard_autonomy::mission::VisionMonitor monitor{detector};
    const onboard_autonomy::mission::TimePoint start{};

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
    onboard_autonomy::mission::VisionMonitor monitor{
        detector,
        {
            .required_consecutive_observations = 3,
            .loss_timeout = 500ms,
            .position_smoothing_factor = 0.5,
            .minimum_decision_margin = 20.0,
        },
    };
    const onboard_autonomy::mission::TimePoint start{};

    monitor.process(empty_frame(1), start);
    monitor.process(empty_frame(2), start + 20ms);
    const auto& current_targets = monitor.process(empty_frame(3), start + 40ms);
    const auto snapshot = monitor.snapshot(start + 40ms);

    require(
        snapshot.target_track.phase ==
                onboard_autonomy::mission::TargetTrackPhase::tracking &&
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

onboard_autonomy::mission::CameraCalibration test_calibration() {
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

    const auto recovered = onboard_autonomy::mission::cv::undistort_image_point(
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

} // namespace

void run_vision_monitor_tests() {
    vision_monitor_tracks_processing_and_detections();
    vision_monitor_exposes_the_smoothed_confirmed_track();
    distortion_round_trip_recovers_undistorted_point();
}
