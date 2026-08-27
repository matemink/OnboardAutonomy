#pragma once

#include "onboard_autonomy/mission/flight/VehicleState.hpp"

#include <chrono>
#include <optional>

namespace onboard_autonomy::mission {

struct AerialYawControllerConfig {
    static constexpr double kDefaultProportionalGainPerSecond = 2.0;
    static constexpr double kDefaultVelocitySmoothingFactor = 0.35;
    static constexpr double kDefaultFeedForwardGain = 1.0;
    static constexpr double kDefaultMaximumYawRateDegreesPerSecond = 90.0;
    static constexpr double kDefaultHorizontalDeadbandRatio = 0.08;
    static constexpr double kDefaultHorizontalFovRadians = 2.0;
    static constexpr auto kDefaultMaximumObservationInterval =
        std::chrono::milliseconds{500};

    double proportional_gain_per_second{kDefaultProportionalGainPerSecond};
    double velocity_smoothing_factor{kDefaultVelocitySmoothingFactor};
    double feed_forward_gain{kDefaultFeedForwardGain};
    double maximum_yaw_rate_degrees_per_second{
        kDefaultMaximumYawRateDegreesPerSecond};
    double horizontal_deadband_ratio{kDefaultHorizontalDeadbandRatio};
    double horizontal_fov_radians{kDefaultHorizontalFovRadians};
    std::chrono::milliseconds maximum_observation_interval{
        kDefaultMaximumObservationInterval};
};

struct AerialYawControl {
    double proportional_rate_degrees_per_second{};
    double feed_forward_rate_degrees_per_second{};
    double yaw_rate_degrees_per_second{};
};

class AerialYawController {
  public:
    explicit AerialYawController(const AerialYawControllerConfig& config = {});

    [[nodiscard]] AerialYawControl update(double horizontal_error,
        double actual_yaw_rate_degrees_per_second,
        mission::TimePoint observed_at);
    void observe_while_holding(double horizontal_error,
        mission::TimePoint observed_at);
    void reset();

  private:
    [[nodiscard]] double update_target_motion(double horizontal_error,
        double applied_yaw_rate_degrees_per_second,
        mission::TimePoint observed_at);

    AerialYawControllerConfig config_;
    std::optional<double> previous_angle_error_degrees_;
    std::optional<mission::TimePoint> previous_observed_at_;
    double smoothed_target_rate_degrees_per_second_{0.0};
};

} // namespace onboard_autonomy::mission
