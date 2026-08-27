#include "onboard_autonomy/mission/autonomy/AerialYawController.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace onboard_autonomy::mission {
namespace {

constexpr double kMinimumHorizontalError = -1.0;
constexpr double kMaximumHorizontalError = 1.0;
constexpr double kMinimumTargetMotionRateDegreesPerSecond = 0.1;
constexpr double kRadiansToDegrees = 180.0 / std::numbers::pi;

} // namespace

AerialYawController::AerialYawController(
    const AerialYawControllerConfig& config)
    : config_(config) {
    if (!std::isfinite(config_.proportional_gain_per_second) ||
        config_.proportional_gain_per_second < 0.0 ||
        !std::isfinite(config_.velocity_smoothing_factor) ||
        config_.velocity_smoothing_factor <= 0.0 ||
        config_.velocity_smoothing_factor > 1.0 ||
        !std::isfinite(config_.feed_forward_gain) ||
        config_.feed_forward_gain < 0.0 || config_.feed_forward_gain > 1.0 ||
        !std::isfinite(config_.maximum_yaw_rate_degrees_per_second) ||
        config_.maximum_yaw_rate_degrees_per_second <= 0.0 ||
        !std::isfinite(config_.horizontal_deadband_ratio) ||
        config_.horizontal_deadband_ratio < 0.0 ||
        config_.horizontal_deadband_ratio >= 1.0 ||
        !std::isfinite(config_.horizontal_fov_radians) ||
        config_.horizontal_fov_radians <= 0.0 ||
        config_.maximum_observation_interval <=
            std::chrono::milliseconds::zero()) {
        throw std::invalid_argument("invalid aerial yaw controller config");
    }
}

AerialYawControl AerialYawController::update(const double horizontal_error,
    const double actual_yaw_rate_degrees_per_second,
    const mission::TimePoint observed_at) {
    const auto angle_error_degrees = update_target_motion(horizontal_error,
        actual_yaw_rate_degrees_per_second,
        observed_at);

    const auto proportional_rate =
        std::abs(horizontal_error) <= config_.horizontal_deadband_ratio
            ? 0.0
            : config_.proportional_gain_per_second * angle_error_degrees;
    const auto feed_forward_rate =
        config_.feed_forward_gain * smoothed_target_rate_degrees_per_second_;
    const bool target_is_approaching_center =
        std::abs(feed_forward_rate) >=
            kMinimumTargetMotionRateDegreesPerSecond &&
        proportional_rate * feed_forward_rate < 0.0;
    const auto requested_yaw_rate = target_is_approaching_center
                                        ? 0.0
                                        : proportional_rate + feed_forward_rate;
    const auto yaw_rate = std::clamp(requested_yaw_rate,
        -config_.maximum_yaw_rate_degrees_per_second,
        config_.maximum_yaw_rate_degrees_per_second);
    return {
        .proportional_rate_degrees_per_second = proportional_rate,
        .feed_forward_rate_degrees_per_second = feed_forward_rate,
        .yaw_rate_degrees_per_second = yaw_rate,
    };
}

void AerialYawController::observe_while_holding(const double horizontal_error,
    const mission::TimePoint observed_at) {
    static_cast<void>(update_target_motion(horizontal_error, 0.0, observed_at));
}

double AerialYawController::update_target_motion(const double horizontal_error,
    const double applied_yaw_rate_degrees_per_second,
    const mission::TimePoint observed_at) {
    if (!std::isfinite(horizontal_error) ||
        horizontal_error < kMinimumHorizontalError ||
        horizontal_error > kMaximumHorizontalError ||
        !std::isfinite(applied_yaw_rate_degrees_per_second)) {
        throw std::invalid_argument("invalid aerial target horizontal error");
    }

    const auto half_fov_degrees =
        config_.horizontal_fov_radians * kRadiansToDegrees / 2.0;
    const auto angle_error_degrees = horizontal_error * half_fov_degrees;

    if (previous_angle_error_degrees_.has_value() &&
        previous_observed_at_.has_value() &&
        observed_at > *previous_observed_at_) {
        const auto interval = observed_at - *previous_observed_at_;
        if (interval <= config_.maximum_observation_interval) {
            const auto seconds =
                std::chrono::duration<double>(interval).count();
            const auto image_error_rate =
                (angle_error_degrees - *previous_angle_error_degrees_) /
                seconds;
            const auto measured_target_rate = std::clamp(
                applied_yaw_rate_degrees_per_second + image_error_rate,
                -config_.maximum_yaw_rate_degrees_per_second,
                config_.maximum_yaw_rate_degrees_per_second);
            const auto smoothing = config_.velocity_smoothing_factor;
            smoothed_target_rate_degrees_per_second_ =
                smoothing * measured_target_rate +
                (1.0 - smoothing) * smoothed_target_rate_degrees_per_second_;
        } else {
            smoothed_target_rate_degrees_per_second_ = 0.0;
        }
    } else {
        smoothed_target_rate_degrees_per_second_ = 0.0;
    }
    previous_angle_error_degrees_ = angle_error_degrees;
    previous_observed_at_ = observed_at;
    return angle_error_degrees;
}

void AerialYawController::reset() {
    previous_angle_error_degrees_.reset();
    previous_observed_at_.reset();
    smoothed_target_rate_degrees_per_second_ = 0.0;
}

} // namespace onboard_autonomy::mission
