#include "onboard_autonomy/mission/safety/SafetySupervisor.hpp"

#include <cmath>

namespace onboard_autonomy::mission {
namespace {

bool valid_target(const mission::BodyFramePosition& target) {
    return std::isfinite(target.forward_m) && std::isfinite(target.right_m) &&
           std::isfinite(target.down_m) && target.down_m > 0.0;
}

} // namespace

SupervisedMotion SafetySupervisor::supervise(const WorldState& world,
    const std::optional<DesiredMotion>& desired,
    const mission::TimePoint now) const {
    if (!desired.has_value()) {
        return {
            .status = MotionSafetyStatus::no_intent,
            .approved = std::nullopt,
            .detail = "waiting for a desired motion",
        };
    }
    if (!world.flight_controller_connected) {
        return {
            .status = MotionSafetyStatus::flight_controller_disconnected,
            .approved = std::nullopt,
            .detail = "flight-controller heartbeat is stale",
        };
    }
    if (!world.vehicle_armed) {
        return {
            .status = MotionSafetyStatus::vehicle_disarmed,
            .approved = std::nullopt,
            .detail = "vehicle is disarmed",
        };
    }
    if (now > desired->valid_until) {
        return {
            .status = MotionSafetyStatus::stale_intent,
            .approved = std::nullopt,
            .detail = "desired motion expired",
        };
    }
    if (!valid_target(desired->landing_target)) {
        return {
            .status = MotionSafetyStatus::invalid_target,
            .approved = std::nullopt,
            .detail = "landing target is invalid",
        };
    }

    return {
        .status = MotionSafetyStatus::allowed,
        .approved = desired,
        .detail = "motion intent allowed",
    };
}

} // namespace onboard_autonomy::mission
