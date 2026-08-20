#include "onboard_autonomy/mission/cv/tracking/TargetTracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace onboard_autonomy::mission {
namespace {

bool valid_observation(const mission::TargetObservation& observation,
    const double minimum_decision_margin) {
    if (!observation.pose.has_value() || observation.corrected_bits != 0 ||
        !std::isfinite(observation.decision_margin) ||
        observation.decision_margin < minimum_decision_margin) {
        return false;
    }

    const auto& position = observation.pose->position;
    return std::isfinite(position.right_m) && std::isfinite(position.down_m) &&
           std::isfinite(position.forward_m) && position.forward_m > 0.0;
}

mission::CameraFramePosition smooth_position(
    const mission::CameraFramePosition& previous,
    const mission::CameraFramePosition& measurement,
    const double factor) {
    const double retained = 1.0 - factor;
    return {
        .right_m = retained * previous.right_m + factor * measurement.right_m,
        .down_m = retained * previous.down_m + factor * measurement.down_m,
        .forward_m =
            retained * previous.forward_m + factor * measurement.forward_m,
    };
}

const mission::TargetObservation* best_observation(
    const std::span<const mission::TargetObservation> observations,
    const double minimum_decision_margin,
    const std::optional<std::int32_t> target_id) {
    const mission::TargetObservation* selected = nullptr;
    for (const auto& observation : observations) {
        if (!valid_observation(observation, minimum_decision_margin) ||
            (target_id.has_value() && observation.id != *target_id)) {
            continue;
        }
        if (selected == nullptr ||
            observation.decision_margin > selected->decision_margin) {
            selected = &observation;
        }
    }
    return selected;
}

} // namespace

TargetTracker::TargetTracker(const TargetTrackerConfig& config)
    : config_(config) {
    if (config_.required_consecutive_observations == 0U) {
        throw std::invalid_argument(
            "target tracker requires at least one observation");
    }
    if (config_.loss_timeout <= std::chrono::milliseconds::zero()) {
        throw std::invalid_argument(
            "target tracker loss timeout must be positive");
    }
    if (!std::isfinite(config_.position_smoothing_factor) ||
        config_.position_smoothing_factor <= 0.0 ||
        config_.position_smoothing_factor > 1.0) {
        throw std::invalid_argument(
            "target tracker smoothing factor must be in (0, 1]");
    }
    if (!std::isfinite(config_.minimum_decision_margin) ||
        config_.minimum_decision_margin < 0.0) {
        throw std::invalid_argument(
            "target tracker minimum decision margin must be non-negative");
    }
}

void TargetTracker::update(
    const std::span<const mission::TargetObservation> observations,
    const mission::TimePoint now) {
    if (expired(now)) {
        reset();
    }

    const auto* selected = best_observation(observations,
        config_.minimum_decision_margin,
        target_id_);

    if (selected == nullptr && target_id_.has_value()) {
        if (consecutive_observations_ <
            config_.required_consecutive_observations) {
            reset();
        }
        return;
    }
    if (selected == nullptr) {
        return;
    }

    if (!selected->pose.has_value()) {
        return;
    }
    const auto measurement = selected->pose->position;
    if (!target_id_.has_value()) {
        target_id_ = selected->id;
        consecutive_observations_ = 1U;
        accepted_observations_ = 1U;
        position_ = measurement;
    } else {
        if (consecutive_observations_ <
            std::numeric_limits<std::uint32_t>::max()) {
            ++consecutive_observations_;
        }
        ++accepted_observations_;
        position_ = smooth_position(position_.value_or(measurement),
            measurement,
            config_.position_smoothing_factor);
    }
    last_observed_at_ = now;
    latest_decision_margin_ = selected->decision_margin;
}

TargetTrackSnapshot TargetTracker::snapshot(
    const mission::TimePoint now) const {
    TargetTrackSnapshot result{
        .phase = TargetTrackPhase::searching,
        .target_id = std::nullopt,
        .consecutive_observations = 0U,
        .required_observations = config_.required_consecutive_observations,
        .accepted_observations = 0U,
        .observation_age_ms = std::nullopt,
        .latest_decision_margin = std::nullopt,
        .position = std::nullopt,
    };
    if (!target_id_.has_value() || expired(now)) {
        return result;
    }

    result.phase =
        consecutive_observations_ >= config_.required_consecutive_observations
            ? TargetTrackPhase::tracking
            : TargetTrackPhase::acquiring;
    result.target_id = target_id_;
    result.consecutive_observations = consecutive_observations_;
    result.accepted_observations = accepted_observations_;
    result.latest_decision_margin = latest_decision_margin_;
    result.position = position_;
    if (last_observed_at_.has_value()) {
        const auto age = now >= *last_observed_at_
                             ? now - *last_observed_at_
                             : mission::Clock::duration::zero();
        result.observation_age_ms =
            std::chrono::duration<double, std::milli>(age).count();
    }
    return result;
}

bool TargetTracker::expired(const mission::TimePoint now) const {
    return last_observed_at_.has_value() && now >= *last_observed_at_ &&
           now - *last_observed_at_ > config_.loss_timeout;
}

void TargetTracker::reset() {
    target_id_.reset();
    consecutive_observations_ = 0U;
    accepted_observations_ = 0U;
    last_observed_at_.reset();
    latest_decision_margin_.reset();
    position_.reset();
}

} // namespace onboard_autonomy::mission
