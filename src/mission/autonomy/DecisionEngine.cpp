#include "onboard_autonomy/mission/autonomy/DecisionEngine.hpp"

namespace onboard_autonomy::mission {

std::optional<DesiredMotion> DecisionEngine::decide(
    const WorldState& world) const {
    if (!world.vehicle_system_id.has_value() ||
        !world.landing_target.has_value()) {
        return std::nullopt;
    }

    return DesiredMotion{
        .type = DesiredMotionType::precision_land,
        .vehicle_system_id = *world.vehicle_system_id,
        .landing_target = *world.landing_target,
        .created_at = world.observed_at,
        .valid_until = world.observed_at + kIntentLifetime,
    };
}

} // namespace onboard_autonomy::mission
