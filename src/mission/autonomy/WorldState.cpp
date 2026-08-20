#include "onboard_autonomy/mission/autonomy/WorldState.hpp"

#include <utility>

namespace onboard_autonomy::mission {

WorldState make_world_state(const mission::VehicleSnapshot& vehicle,
    std::optional<mission::BodyFramePosition> landing_target,
    const mission::TimePoint observed_at) {
    return {
        .flight_controller_connected = vehicle.connected,
        .vehicle_armed = vehicle.armed,
        .vehicle_system_id = vehicle.system_id,
        .landing_target = landing_target,
        .observed_at = observed_at,
    };
}

} // namespace onboard_autonomy::mission
