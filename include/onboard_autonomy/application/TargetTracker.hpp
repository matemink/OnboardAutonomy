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
    static constexpr std::uint32_t kDefaultRequiredObservations = 3;
    static constexpr auto kDefaultLossTimeout = std::chrono::milliseconds{500};
    static constexpr double kDefaultPositionSmoothingFactor = 1.0;
    static constexpr double kDefaultMinimumDecisionMargin = 20.0;

    std::uint32_t required_consecutive_observations{
        kDefaultRequiredObservations};
    std::chrono::milliseconds loss_timeout{kDefaultLossTimeout};
    double position_smoothing_factor{kDefaultPositionSmoothingFactor};
    double minimum_decision_margin{kDefaultMinimumDecisionMargin};
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

    void update(std::span<const domain::TargetObservation> observations,
        domain::TimePoint now);
    [[nodiscard]] TargetTrackSnapshot snapshot(domain::TimePoint now) const;

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

} // namespace onboard_autonomy::application
