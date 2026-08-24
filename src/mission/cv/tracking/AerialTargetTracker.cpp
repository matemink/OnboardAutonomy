#include "onboard_autonomy/mission/cv/tracking/AerialTargetTracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace onboard_autonomy::mission {
namespace {

constexpr double kNormalizedHorizontalSpan = 2.0;
constexpr double kNormalizedFrameCenter = 1.0;

} // namespace

std::optional<AerialTargetTracker::Candidate>
AerialTargetTracker::make_candidate(
    const mission::TargetObservation& observation,
    const std::uint32_t frame_width,
    const std::uint32_t frame_height) const {
    if (frame_width == 0U || frame_height == 0U ||
        !std::isfinite(observation.decision_margin) ||
        observation.decision_margin < config_.minimum_confidence_percent) {
        return std::nullopt;
    }

    double left = std::numeric_limits<double>::infinity();
    double top = std::numeric_limits<double>::infinity();
    double right = -std::numeric_limits<double>::infinity();
    double bottom = -std::numeric_limits<double>::infinity();
    for (const auto& corner : observation.corners) {
        if (!std::isfinite(corner.x_px) || !std::isfinite(corner.y_px)) {
            return std::nullopt;
        }
        left = std::min(left, corner.x_px);
        top = std::min(top, corner.y_px);
        right = std::max(right, corner.x_px);
        bottom = std::max(bottom, corner.y_px);
    }
    if (!(right > left) || !(bottom > top)) {
        return std::nullopt;
    }

    const auto width = static_cast<double>(frame_width);
    const auto height = static_cast<double>(frame_height);
    const auto center_x = (left + right) / 2.0;
    const auto center_y = (top + bottom) / 2.0;
    const auto center_x_ratio = center_x / width;
    const auto center_y_ratio = center_y / height;
    if (center_x_ratio < 0.0 || center_x_ratio > 1.0 || center_y_ratio < 0.0 ||
        center_y_ratio > 1.0) {
        return std::nullopt;
    }

    return Candidate{
        .center_x_ratio = center_x_ratio,
        .center_y_ratio = center_y_ratio,
        .width_ratio = (right - left) / width,
        .height_ratio = (bottom - top) / height,
        .confidence_percent = observation.decision_margin,
    };
}

double AerialTargetTracker::center_distance(const Candidate& left,
    const Candidate& right) {
    return std::hypot(left.center_x_ratio - right.center_x_ratio,
        left.center_y_ratio - right.center_y_ratio);
}

AerialTargetTracker::AerialTargetTracker(
    const AerialTargetTrackerConfig& config)
    : config_(config) {
    if (config_.required_consecutive_observations == 0U ||
        config_.loss_timeout <= std::chrono::milliseconds::zero() ||
        !std::isfinite(config_.minimum_confidence_percent) ||
        config_.minimum_confidence_percent < 0.0 ||
        config_.minimum_confidence_percent > 100.0 ||
        !std::isfinite(config_.maximum_center_jump_ratio) ||
        config_.maximum_center_jump_ratio <= 0.0 ||
        config_.maximum_center_jump_ratio > 1.0) {
        throw std::invalid_argument(
            "invalid aerial target tracker configuration");
    }
}

void AerialTargetTracker::update(
    const std::span<const mission::TargetObservation> observations,
    const std::uint32_t frame_width,
    const std::uint32_t frame_height,
    const std::uint64_t frame_sequence,
    const mission::TimePoint now) {
    if (last_frame_sequence_.has_value() &&
        frame_sequence <= *last_frame_sequence_) {
        return;
    }
    if (expired(now)) {
        reset();
    }

    std::vector<Candidate> candidates;
    candidates.reserve(observations.size());
    for (const auto& observation : observations) {
        if (auto candidate =
                make_candidate(observation, frame_width, frame_height)) {
            candidates.push_back(*candidate);
        }
    }

    const Candidate* selected = nullptr;
    if (candidate_.has_value()) {
        double best_distance = config_.maximum_center_jump_ratio;
        for (const auto& candidate : candidates) {
            const auto distance = center_distance(*candidate_, candidate);
            if (distance <= best_distance) {
                selected = &candidate;
                best_distance = distance;
            }
        }
    } else {
        for (const auto& candidate : candidates) {
            if (selected == nullptr ||
                candidate.confidence_percent > selected->confidence_percent) {
                selected = &candidate;
            }
        }
    }

    if (selected != nullptr) {
        accept(*selected, frame_sequence, now);
        return;
    }

    last_frame_sequence_ = frame_sequence;
}

AerialTargetTrackSnapshot AerialTargetTracker::snapshot(
    const mission::TimePoint now) const {
    AerialTargetTrackSnapshot result{
        .phase = AerialTargetTrackPhase::searching,
        .consecutive_observations = 0U,
        .required_observations = config_.required_consecutive_observations,
        .accepted_observations = 0U,
        .observation_age_ms = std::nullopt,
        .confidence_percent = std::nullopt,
        .horizontal_error = std::nullopt,
        .center_y_ratio = std::nullopt,
        .width_ratio = std::nullopt,
        .height_ratio = std::nullopt,
    };
    if (!candidate_.has_value() || expired(now)) {
        return result;
    }

    result.phase =
        consecutive_observations_ >= config_.required_consecutive_observations
            ? AerialTargetTrackPhase::tracking
            : AerialTargetTrackPhase::acquiring;
    result.consecutive_observations = consecutive_observations_;
    result.accepted_observations = accepted_observations_;
    result.confidence_percent = candidate_->confidence_percent;
    result.horizontal_error =
        kNormalizedHorizontalSpan * candidate_->center_x_ratio -
        kNormalizedFrameCenter;
    result.center_y_ratio = candidate_->center_y_ratio;
    result.width_ratio = candidate_->width_ratio;
    result.height_ratio = candidate_->height_ratio;
    if (last_observed_at_.has_value()) {
        const auto age = now >= *last_observed_at_
                             ? now - *last_observed_at_
                             : mission::Clock::duration::zero();
        result.observation_age_ms =
            std::chrono::duration<double, std::milli>(age).count();
    }
    return result;
}

bool AerialTargetTracker::expired(const mission::TimePoint now) const {
    return last_observed_at_.has_value() && now >= *last_observed_at_ &&
           now - *last_observed_at_ > config_.loss_timeout;
}

void AerialTargetTracker::accept(const Candidate& candidate,
    const std::uint64_t frame_sequence,
    const mission::TimePoint now) {
    candidate_ = candidate;
    if (consecutive_observations_ < std::numeric_limits<std::uint32_t>::max()) {
        ++consecutive_observations_;
    }
    ++accepted_observations_;
    last_frame_sequence_ = frame_sequence;
    last_observed_at_ = now;
}

void AerialTargetTracker::reset() {
    candidate_.reset();
    consecutive_observations_ = 0U;
    accepted_observations_ = 0U;
    last_frame_sequence_.reset();
    last_observed_at_.reset();
}

} // namespace onboard_autonomy::mission
