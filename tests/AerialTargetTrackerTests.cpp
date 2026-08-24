#include "TestCases.hpp"

#include "onboard_autonomy/mission/cv/tracking/AerialTargetTracker.hpp"

#include <array>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using onboard_autonomy::mission::AerialTargetTracker;
using onboard_autonomy::mission::AerialTargetTrackPhase;
using onboard_autonomy::mission::TargetObservation;
using onboard_autonomy::mission::TimePoint;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

TargetObservation airplane(const double center_x,
    const double center_y,
    const double confidence = 80.0,
    const double width = 80.0,
    const double height = 40.0) {
    const auto left = center_x - width / 2.0;
    const auto right = center_x + width / 2.0;
    const auto top = center_y - height / 2.0;
    const auto bottom = center_y + height / 2.0;
    return {
        .id = 4,
        .family = "airplane",
        .center = {.x_px = center_x, .y_px = center_y},
        .corners = {{
            {.x_px = left, .y_px = top},
            {.x_px = right, .y_px = top},
            {.x_px = right, .y_px = bottom},
            {.x_px = left, .y_px = bottom},
        }},
        .decision_margin = confidence,
        .pose = std::nullopt,
    };
}

void highest_confidence_detection_starts_the_track() {
    AerialTargetTracker tracker;
    const TimePoint start{};
    for (std::uint64_t sequence = 1; sequence <= 3; ++sequence) {
        const std::array observations{
            airplane(150.0, 180.0, 70.0),
            airplane(470.0, 320.0, 92.0),
        };
        tracker.update(observations,
            640,
            480,
            sequence,
            start + std::chrono::milliseconds(sequence * 30));
    }

    const auto track = tracker.snapshot(start + std::chrono::milliseconds(90));
    require(track.phase == AerialTargetTrackPhase::tracking,
        "three consistent frames must acquire the aerial target");
    require(track.horizontal_error.has_value() && *track.horizontal_error > 0.0,
        "acquisition must select the most confident detection");
}

void detection_can_be_acquired_anywhere_in_the_frame() {
    AerialTargetTracker tracker;
    const TimePoint start{};
    for (std::uint64_t sequence = 1; sequence <= 4; ++sequence) {
        const std::array observations{airplane(320.0, 420.0, 95.0)};
        tracker.update(observations,
            640,
            480,
            sequence,
            start + std::chrono::milliseconds(sequence * 30));
    }

    require(tracker.snapshot(start + std::chrono::milliseconds(120)).phase ==
                AerialTargetTrackPhase::tracking,
        "image position must not prevent a valid target track");
}

void short_detector_misses_do_not_discard_acquisition() {
    AerialTargetTracker tracker;
    const TimePoint start{};
    const std::vector<TargetObservation> missing;

    for (std::uint64_t sequence = 1; sequence <= 5; ++sequence) {
        if (sequence % 2U == 0U) {
            tracker.update(missing,
                640,
                480,
                sequence,
                start + std::chrono::milliseconds(sequence * 60));
            continue;
        }

        const std::array observations{
            airplane(300.0 + static_cast<double>(sequence), 180.0),
        };
        tracker.update(observations,
            640,
            480,
            sequence,
            start + std::chrono::milliseconds(sequence * 60));
    }

    require(tracker.snapshot(start + std::chrono::milliseconds(300)).phase ==
                AerialTargetTrackPhase::tracking,
        "short detector misses must not reset consistent acquisition");
}

void require_class_can_start_a_track(const std::int32_t class_id,
    const std::string& class_name) {
    AerialTargetTracker tracker;
    auto observation = airplane(320.0, 180.0, 99.0);
    observation.id = class_id;
    observation.family = class_name;
    const std::array observations{observation};
    const TimePoint start{};
    for (std::uint64_t sequence = 1; sequence <= 4; ++sequence) {
        tracker.update(observations,
            640,
            480,
            sequence,
            start + std::chrono::milliseconds(sequence * 30));
    }

    require(tracker.snapshot(start + std::chrono::milliseconds(120)).phase ==
                AerialTargetTrackPhase::tracking,
        class_name + " detections must be eligible for tracking");
}

void any_forward_detector_class_can_start_a_track() {
    require_class_can_start_a_track(14, "bird");
    require_class_can_start_a_track(33, "kite");
}

void locked_track_uses_continuity_and_expires() {
    AerialTargetTracker tracker;
    const TimePoint start{};
    for (std::uint64_t sequence = 1; sequence <= 3; ++sequence) {
        const std::array observations{airplane(280.0, 170.0)};
        tracker.update(observations,
            640,
            480,
            sequence,
            start + std::chrono::milliseconds(sequence * 30));
    }

    const std::array competing{
        airplane(300.0, 180.0, 65.0),
        airplane(500.0, 370.0, 99.0),
    };
    tracker.update(competing,
        640,
        480,
        4,
        start + std::chrono::milliseconds(120));
    const auto maintained =
        tracker.snapshot(start + std::chrono::milliseconds(120));
    require(maintained.phase == AerialTargetTrackPhase::tracking &&
                maintained.horizontal_error.has_value() &&
                *maintained.horizontal_error < 0.0,
        "tracking must prefer the spatially continuous target");

    const std::vector<TargetObservation> missing;
    tracker.update(missing,
        640,
        480,
        5,
        start + std::chrono::milliseconds(150));
    require(tracker.snapshot(start + std::chrono::milliseconds(400)).phase ==
                AerialTargetTrackPhase::tracking,
        "a short detector miss must preserve the lock");
    require(tracker.snapshot(start + std::chrono::milliseconds(700)).phase ==
                AerialTargetTrackPhase::searching,
        "a stale lock must expire to searching");
}

} // namespace

void run_aerial_target_tracker_tests() {
    highest_confidence_detection_starts_the_track();
    detection_can_be_acquired_anywhere_in_the_frame();
    short_detector_misses_do_not_discard_acquisition();
    any_forward_detector_class_can_start_a_track();
    locked_track_uses_continuity_and_expires();
}
