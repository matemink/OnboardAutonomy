#include "TestCases.hpp"

#include "onboard_autonomy/mission/cv/tracking/TargetTracker.hpp"

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

onboard_autonomy::mission::TargetObservation observation(const std::int32_t id,
    const double right_m,
    const double down_m,
    const double forward_m,
    const double decision_margin = 60.0,
    const std::int32_t corrected_bits = 0) {
    return {
        .id = id,
        .family = "tagStandard41h12",
        .center = {},
        .corners = {},
        .corrected_bits = corrected_bits,
        .decision_margin = decision_margin,
        .pose =
            onboard_autonomy::mission::TargetPose{
                .position =
                    {
                        .right_m = right_m,
                        .down_m = down_m,
                        .forward_m = forward_m,
                    },
                .rotation_tag_to_camera = {},
                .object_space_error = 0.001,
            },
    };
}

onboard_autonomy::mission::TargetTracker tracker() {
    using namespace std::chrono_literals;
    return onboard_autonomy::mission::TargetTracker{
        {
            .required_consecutive_observations = 3,
            .loss_timeout = 500ms,
            .position_smoothing_factor = 0.5,
            .minimum_decision_margin = 20.0,
        },
    };
}

void tracker_requires_confirmation_and_smooths_position() {
    using namespace std::chrono_literals;
    auto target_tracker = tracker();
    const onboard_autonomy::mission::TimePoint start{};

    target_tracker.update(std::vector{observation(0, 0.0, 0.0, 1.0)}, start);
    auto snapshot = target_tracker.snapshot(start);
    require(snapshot.phase ==
                    onboard_autonomy::mission::TargetTrackPhase::acquiring &&
                snapshot.consecutive_observations == 1U,
        "one pose must start acquisition without claiming a lock");

    target_tracker.update(std::vector{observation(0, 1.0, -0.2, 1.2)},
        start + 20ms);
    target_tracker.update(std::vector{observation(0, 1.0, -0.2, 0.8)},
        start + 40ms);
    snapshot = target_tracker.snapshot(start + 40ms);
    require(snapshot.phase ==
                    onboard_autonomy::mission::TargetTrackPhase::tracking &&
                snapshot.position.has_value() &&
                std::abs(snapshot.position->right_m - 0.75) < 1.0e-9 &&
                std::abs(snapshot.position->down_m + 0.15) < 1.0e-9 &&
                std::abs(snapshot.position->forward_m - 0.95) < 1.0e-9,
        "three observations must lock and use exponential smoothing");
}

void tracker_exposes_freshness_and_expires() {
    using namespace std::chrono_literals;
    auto target_tracker = tracker();
    const onboard_autonomy::mission::TimePoint start{};
    const auto tag = std::vector{observation(0, 0.1, 0.2, 1.0)};

    target_tracker.update(tag, start);
    target_tracker.update(tag, start + 20ms);
    target_tracker.update(tag, start + 40ms);
    target_tracker.update({}, start + 300ms);

    auto snapshot = target_tracker.snapshot(start + 340ms);
    require(snapshot.phase ==
                    onboard_autonomy::mission::TargetTrackPhase::tracking &&
                snapshot.observation_age_ms.has_value() &&
                std::abs(*snapshot.observation_age_ms - 300.0) < 0.01,
        "a brief dropout must retain lock while exposing its age");

    snapshot = target_tracker.snapshot(start + 541ms);
    require(snapshot.phase ==
                    onboard_autonomy::mission::TargetTrackPhase::searching &&
                !snapshot.target_id.has_value() &&
                !snapshot.position.has_value(),
        "an expired observation must not remain a usable target");
}

void acquisition_requires_uninterrupted_observations() {
    using namespace std::chrono_literals;
    auto target_tracker = tracker();
    const onboard_autonomy::mission::TimePoint start{};
    const auto tag = std::vector{observation(0, 0.1, 0.2, 1.0)};

    target_tracker.update(tag, start);
    target_tracker.update({}, start + 20ms);
    auto snapshot = target_tracker.snapshot(start + 20ms);
    require(snapshot.phase ==
                onboard_autonomy::mission::TargetTrackPhase::searching,
        "a missing frame during acquisition must reset the streak");

    target_tracker.update(tag, start + 40ms);
    snapshot = target_tracker.snapshot(start + 40ms);
    require(snapshot.phase ==
                    onboard_autonomy::mission::TargetTrackPhase::acquiring &&
                snapshot.consecutive_observations == 1U,
        "acquisition must restart from the next valid observation");
}

void tracker_does_not_jump_between_tags() {
    using namespace std::chrono_literals;
    auto target_tracker = tracker();
    const onboard_autonomy::mission::TimePoint start{};

    target_tracker.update(std::vector{observation(0, 0.0, 0.0, 1.0)}, start);
    target_tracker.update(std::vector{observation(0, 0.0, 0.0, 1.0)},
        start + 20ms);
    target_tracker.update(std::vector{observation(0, 0.0, 0.0, 1.0)},
        start + 40ms);
    target_tracker.update(std::vector{observation(1, 0.4, 0.0, 1.0, 90.0)},
        start + 100ms);
    auto snapshot = target_tracker.snapshot(start + 100ms);
    require(snapshot.target_id == 0 &&
                snapshot.phase ==
                    onboard_autonomy::mission::TargetTrackPhase::tracking,
        "a confirmed track must not switch to a different tag");

    target_tracker.update(std::vector{observation(1, 0.4, 0.0, 1.0, 90.0)},
        start + 541ms);
    snapshot = target_tracker.snapshot(start + 541ms);
    require(snapshot.target_id == 1 &&
                snapshot.phase ==
                    onboard_autonomy::mission::TargetTrackPhase::acquiring,
        "a new tag may acquire only after the previous lock expires");
}

void tracker_rejects_unsafe_pose_observations() {
    auto target_tracker = tracker();
    const onboard_autonomy::mission::TimePoint now{};
    auto missing_pose = observation(0, 0.0, 0.0, 1.0);
    missing_pose.pose = std::nullopt;

    target_tracker.update(
        std::vector{
            observation(0, 0.0, 0.0, 1.0, 19.9),
            observation(0, 0.0, 0.0, 1.0, 60.0, 1),
            observation(0, 0.0, 0.0, -1.0),
            missing_pose,
        },
        now);
    require(target_tracker.snapshot(now).phase ==
                onboard_autonomy::mission::TargetTrackPhase::searching,
        "low-confidence, corrected, missing, or behind-camera poses "
        "must be rejected");
}

} // namespace

void run_target_tracker_tests() {
    tracker_requires_confirmation_and_smooths_position();
    tracker_exposes_freshness_and_expires();
    acquisition_requires_uninterrupted_observations();
    tracker_does_not_jump_between_tags();
    tracker_rejects_unsafe_pose_observations();
}
