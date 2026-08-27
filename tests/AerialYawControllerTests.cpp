#include "TestCases.hpp"

#include "onboard_autonomy/mission/autonomy/AerialYawController.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {

using onboard_autonomy::mission::AerialYawController;
using onboard_autonomy::mission::AerialYawControllerConfig;
using onboard_autonomy::mission::TimePoint;

constexpr double kTolerance = 0.001;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_near(const double actual,
    const double expected,
    const std::string& message) {
    require(std::abs(actual - expected) <= kTolerance, message);
}

AerialYawControllerConfig deterministic_config() {
    return {
        .proportional_gain_per_second = 1.0,
        .velocity_smoothing_factor = 1.0,
        .feed_forward_gain = 1.0,
        .maximum_yaw_rate_degrees_per_second = 180.0,
        .horizontal_deadband_ratio = 0.0,
        .horizontal_fov_radians = 2.0,
        .maximum_observation_interval = std::chrono::seconds(2),
    };
}

void stationary_world_target_has_no_feed_forward() {
    AerialYawController controller{deterministic_config()};
    const TimePoint start{};

    const auto initial = controller.update(0.4, 0.0, start);
    // During 100 ms the first yaw command moves a stationary target 10% closer
    // to the image centre, so its estimated world angular rate remains zero.
    const auto control = controller.update(0.36,
        initial.yaw_rate_degrees_per_second,
        start + std::chrono::milliseconds(100));

    require_near(control.feed_forward_rate_degrees_per_second,
        0.0,
        "stationary target must not produce feed-forward yaw");
    require(control.yaw_rate_degrees_per_second > 0.0,
        "target on the right must produce positive MAVLink yaw");
}

double rate_for_target_motion(const double second_horizontal_error) {
    AerialYawController controller{deterministic_config()};
    const TimePoint start{};
    static_cast<void>(controller.update(0.6, 0.0, start));
    return controller
        .update(second_horizontal_error,
            0.0,
            start + std::chrono::milliseconds(200))
        .feed_forward_rate_degrees_per_second;
}

void faster_target_produces_faster_yaw() {
    const auto slow_rate = rate_for_target_motion(0.7);
    const auto fast_rate = rate_for_target_motion(0.9);

    require(slow_rate > 0.0,
        "rightward target motion must produce positive tracking yaw");
    require(std::abs(fast_rate) > std::abs(slow_rate),
        "faster image motion must produce a larger feed-forward yaw rate");
}

void noisy_motion_is_smoothed() {
    auto config = deterministic_config();
    config.velocity_smoothing_factor = 0.25;
    AerialYawController controller{config};
    const TimePoint start{};

    static_cast<void>(controller.update(0.5, 0.0, start));
    const auto first =
        controller.update(0.3, 0.0, start + std::chrono::milliseconds(100));
    const auto reversal =
        controller.update(0.4, 0.0, start + std::chrono::milliseconds(200));

    require(first.feed_forward_rate_degrees_per_second < 0.0,
        "initial target motion must be measured");
    require(reversal.feed_forward_rate_degrees_per_second < 0.0,
        "a single noisy reversal must not immediately flip yaw direction");
}

void yaw_rate_is_clamped_to_safe_limit() {
    AerialYawController controller;
    const auto control = controller.update(1.0, 0.0, TimePoint{});

    require_near(control.yaw_rate_degrees_per_second,
        AerialYawControllerConfig::kDefaultMaximumYawRateDegreesPerSecond,
        "yaw controller must enforce its configured rate limit");
}

void stale_observation_discards_velocity_history() {
    AerialYawController controller;
    const TimePoint start{};
    static_cast<void>(controller.update(0.6, 0.0, start));

    const auto control =
        controller.update(-0.6, 0.0, start + std::chrono::seconds(1));

    require_near(control.feed_forward_rate_degrees_per_second,
        0.0,
        "stale observation must not create a derivative spike");
}

void reset_discards_lost_target_motion() {
    AerialYawController controller{deterministic_config()};
    const TimePoint start{};
    static_cast<void>(controller.update(0.6, 0.0, start));
    const auto moving =
        controller.update(0.2, 0.0, start + std::chrono::milliseconds(100));
    require(moving.feed_forward_rate_degrees_per_second < 0.0,
        "test setup must establish target motion");

    controller.reset();
    const auto reacquired =
        controller.update(-0.4, 0.0, start + std::chrono::milliseconds(200));

    require_near(reacquired.feed_forward_rate_degrees_per_second,
        0.0,
        "reacquired target must start without stale feed-forward motion");
}

void approaching_target_does_not_trigger_opposite_yaw() {
    AerialYawController controller{deterministic_config()};
    const TimePoint start{};
    controller.observe_while_holding(0.6, start);
    controller.observe_while_holding(0.4,
        start + std::chrono::milliseconds(100));

    const auto approaching =
        controller.update(0.2, 0.0, start + std::chrono::milliseconds(200));
    require_near(approaching.yaw_rate_degrees_per_second,
        0.0,
        "target approaching image centre must not trigger opposite yaw");

    const auto crossed =
        controller.update(-0.1, 0.0, start + std::chrono::milliseconds(300));
    require(crossed.yaw_rate_degrees_per_second < 0.0,
        "yaw must follow target motion after it crosses image centre");
}

void slow_inward_motion_preserves_proportional_correction() {
    AerialYawController controller{deterministic_config()};
    const TimePoint start{};
    controller.observe_while_holding(0.6, start);

    const auto control =
        controller.update(0.59, 0.0, start + std::chrono::milliseconds(100));

    require(control.feed_forward_rate_degrees_per_second < 0.0,
        "inward target motion must oppose the proportional correction");
    require(control.yaw_rate_degrees_per_second > 0.0,
        "slow inward motion must not discard proportional centering");
    require(control.yaw_rate_degrees_per_second <
                control.proportional_rate_degrees_per_second,
        "inward feed-forward must reduce the proportional correction");
}

} // namespace

void run_aerial_yaw_controller_tests() {
    stationary_world_target_has_no_feed_forward();
    faster_target_produces_faster_yaw();
    noisy_motion_is_smoothed();
    yaw_rate_is_clamped_to_safe_limit();
    stale_observation_discards_velocity_history();
    reset_discards_lost_target_motion();
    approaching_target_does_not_trigger_opposite_yaw();
    slow_inward_motion_preserves_proportional_correction();
}
