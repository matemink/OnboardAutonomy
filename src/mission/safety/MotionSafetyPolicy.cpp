#include "onboard_autonomy/mission/safety/MotionSafetyPolicy.hpp"

namespace onboard_autonomy::mission {

MotionSafetyDecision evaluate_motion_safety(
    const RuntimeEnvironment environment,
    const MavlinkTransport transport,
    const bool motion_requested) {
    if (environment == RuntimeEnvironment::sitl &&
        transport != MavlinkTransport::udp) {
        return {
            .configuration_valid = false,
            .motion_commands_allowed = false,
            .reason = "--sitl requires UDP transport",
        };
    }

    if (!motion_requested) {
        return {
            .configuration_valid = true,
            .motion_commands_allowed = false,
            .reason = "observation-only mode",
        };
    }

    if (environment != RuntimeEnvironment::sitl) {
        return {
            .configuration_valid = false,
            .motion_commands_allowed = false,
            .reason = "motion commands require explicit --sitl mode",
        };
    }

    return {
        .configuration_valid = true,
        .motion_commands_allowed = true,
        .reason = "explicit SITL motion mode",
    };
}

} // namespace onboard_autonomy::mission
