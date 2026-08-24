#pragma once

#include "onboard_autonomy/mission/cv/detection/TargetObservation.hpp"
#include "onboard_autonomy/mission/flight/VehicleState.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <span>

namespace onboard_autonomy::mission {

enum class AerialTargetTrackPhase {
    searching,
    acquiring,
    tracking,
};

struct AerialTargetTrackerConfig {
    static constexpr std::int32_t kDefaultTargetClassId = 4;
    static constexpr std::uint32_t kDefaultRequiredObservations = 3;
    static constexpr auto kDefaultLossTimeout = std::chrono::milliseconds{500};
    static constexpr double kDefaultMinimumConfidencePercent = 35.0;
    static constexpr double kDefaultMaximumCenterJumpRatio = 0.18;

    std::int32_t target_class_id{kDefaultTargetClassId};
    std::uint32_t required_consecutive_observations{
        kDefaultRequiredObservations};
    std::chrono::milliseconds loss_timeout{kDefaultLossTimeout};
    double minimum_confidence_percent{kDefaultMinimumConfidencePercent};
    double maximum_center_jump_ratio{kDefaultMaximumCenterJumpRatio};
};

struct AerialTargetTrackSnapshot {
    AerialTargetTrackPhase phase{AerialTargetTrackPhase::searching};
    std::uint32_t consecutive_observations{0};
    std::uint32_t required_observations{0};
    std::uint64_t accepted_observations{0};
    std::optional<double> observation_age_ms;
    std::optional<double> confidence_percent;
    std::optional<double> horizontal_error;
    std::optional<double> center_y_ratio;
    std::optional<double> width_ratio;
    std::optional<double> height_ratio;
};

class AerialTargetTracker {
  public:
    explicit AerialTargetTracker(const AerialTargetTrackerConfig& config = {});

    void update(std::span<const mission::TargetObservation> observations,
        std::uint32_t frame_width,
        std::uint32_t frame_height,
        std::uint64_t frame_sequence,
        mission::TimePoint now);
    [[nodiscard]] AerialTargetTrackSnapshot snapshot(
        mission::TimePoint now) const;

  private:
    struct Candidate {
        double center_x_ratio{};
        double center_y_ratio{};
        double width_ratio{};
        double height_ratio{};
        double confidence_percent{};
    };

    [[nodiscard]] std::optional<Candidate> make_candidate(
        const mission::TargetObservation& observation,
        std::uint32_t frame_width,
        std::uint32_t frame_height) const;
    [[nodiscard]] static double center_distance(const Candidate& left,
        const Candidate& right);
    [[nodiscard]] bool expired(mission::TimePoint now) const;
    void accept(const Candidate& candidate,
        std::uint64_t frame_sequence,
        mission::TimePoint now);
    void reset();

    AerialTargetTrackerConfig config_;
    std::optional<Candidate> candidate_;
    std::uint32_t consecutive_observations_{0};
    std::uint64_t accepted_observations_{0};
    std::optional<std::uint64_t> last_frame_sequence_;
    std::optional<mission::TimePoint> last_observed_at_;
};

} // namespace onboard_autonomy::mission
