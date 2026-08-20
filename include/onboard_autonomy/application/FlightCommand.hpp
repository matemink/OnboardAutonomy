#pragma once

#include <cstdint>

namespace onboard_autonomy::application {

enum class FlightAction {
    invalid,
    set_guided_mode,
    arm,
    takeoff,
    return_to_launch,
    land,
    landing_target,
};

enum class FlightCommandAckOutcome {
    accepted,
    in_progress,
    rejected,
};

struct FlightActionRequest {
    FlightAction action{FlightAction::invalid};
    std::uint8_t vehicle_system_id{0};
    std::uint8_t confirmation{0};
    double altitude_m{0.0};
    double x_m{0.0};
    double y_m{0.0};
    double z_m{0.0};
    std::uint64_t time_usec{0};
};

} // namespace onboard_autonomy::application
