#pragma once

#include "onboard_autonomy/mission/cv/extrinsics/TargetTransform.hpp"
#include "onboard_autonomy/mission/flight/VehicleState.hpp"

#include <cstdint>
#include <optional>

namespace onboard_autonomy::mission {

struct WorldState {
    bool flight_controller_connected{false};
    bool vehicle_armed{false};
    std::optional<std::uint8_t> vehicle_system_id;
    std::optional<mission::BodyFramePosition> landing_target;
    mission::TimePoint observed_at;
};

[[nodiscard]] WorldState make_world_state(
    const mission::VehicleSnapshot& vehicle,
    std::optional<mission::BodyFramePosition> landing_target,
    mission::TimePoint observed_at);

} // namespace onboard_autonomy::mission
