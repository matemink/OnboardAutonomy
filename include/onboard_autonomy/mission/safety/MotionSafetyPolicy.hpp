#pragma once

#include <string_view>

namespace onboard_autonomy::mission {

enum class RuntimeEnvironment {
    hardware_or_unknown,
    sitl,
};

enum class MavlinkTransport {
    udp,
    serial,
};

struct MotionSafetyDecision {
    bool configuration_valid{false};
    bool motion_commands_allowed{false};
    std::string_view reason;
};

[[nodiscard]] MotionSafetyDecision evaluate_motion_safety(
    RuntimeEnvironment environment,
    MavlinkTransport transport,
    bool motion_requested);

} // namespace onboard_autonomy::mission
