#pragma once

#include "onboard_autonomy/mission/safety/MotionSafetyStatus.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace onboard_autonomy::mission {

enum class AutonomyRuntimePhase {
    disabled,
    idle,
    waiting_for_startup,
    active,
    suspended,
    returning_to_launch,
    completed,
    failed,
};

struct AutonomyRuntimeSnapshot {
    AutonomyRuntimePhase phase{AutonomyRuntimePhase::disabled};
    std::string detail{"Autonomy runtime disabled"};
    MotionSafetyStatus motion_safety_status{MotionSafetyStatus::no_intent};
    std::optional<std::uint8_t> failure_result;
    std::optional<double> aerial_horizontal_error;
    std::optional<double> aerial_proportional_rate_degrees_per_second;
    std::optional<double> aerial_feed_forward_rate_degrees_per_second;
    std::optional<double> aerial_commanded_yaw_rate_degrees_per_second;
};

} // namespace onboard_autonomy::mission
