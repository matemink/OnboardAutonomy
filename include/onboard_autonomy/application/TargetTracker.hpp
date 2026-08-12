#pragma once

#include "onboard_autonomy/domain/TargetObservation.hpp"
#include "onboard_autonomy/domain/VehicleState.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <span>

namespace onboard_autonomy::application {

enum class TargetTrackPhase {
    searching,
    acquiring,
    tracking,
};

struct TargetTrackerConfig {
    std::uint32_t required_consecutive_observations{3};
    std::chrono::milliseconds loss_timeout{500};
    double position_smoothing_factor{1.0};
    double minimum_decision_margin{20.0};
};

struct TargetTrackSnapshot {
    TargetTrackPhase phase{TargetTrackPhase::searching};
    std::optional<std::int32_t> target_id;
    std::uint32_t consecutive_observations{0};
    std::uint32_t required_observations{0};
    std::uint64_t accepted_observations{0};
    std::optional<double> observation_age_ms;
    std::optional<double> latest_decision_margin;
    std::optional<domain::CameraFramePosition> position;
};

class TargetTracker {
public:
    explicit TargetTracker(const TargetTrackerConfig& config = {});

    void update(
        std::span<const domain::TargetObservation> observations,
        domain::TimePoint now
    );
    [[nodiscard]] TargetTrackSnapshot snapshot(
        domain::TimePoint now
    ) const;

private:
    [[nodiscard]] bool expired(domain::TimePoint now) const;
    void reset();

    TargetTrackerConfig config_;
    std::optional<std::int32_t> target_id_;
    std::uint32_t consecutive_observations_{0};
    std::uint64_t accepted_observations_{0};
    std::optional<domain::TimePoint> last_observed_at_;
    std::optional<double> latest_decision_margin_;
    std::optional<domain::CameraFramePosition> position_;
};

}  // namespace onboard_autonomy::application
