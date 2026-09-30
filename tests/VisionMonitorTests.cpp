#include "TestCases.hpp"

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

                .confidence_percent = 90.0,

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

} // namespace

void run_vision_monitor_tests() {
    vision_monitor_tracks_processing_and_detections();
}
