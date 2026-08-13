#include "onboard_autonomy/application/WorldState.hpp"

#include <utility>

namespace onboard_autonomy::application {

WorldState make_world_state(const domain::VehicleSnapshot& vehicle,
    std::optional<domain::BodyFramePosition> landing_target,
    const domain::TimePoint observed_at) {
    return {
        .flight_controller_connected = vehicle.connected,
        .vehicle_armed = vehicle.armed,
        .vehicle_system_id = vehicle.system_id,
        .landing_target = landing_target,
        .observed_at = observed_at,
    };
}

} // namespace onboard_autonomy::application
