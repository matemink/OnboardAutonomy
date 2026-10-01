#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace onboard_autonomy::mission {

enum class FlightStartupPhase {
    disabled,
    idle,
    waiting_for_vehicle,
    waiting_for_readiness,
    setting_guided,
    arming,
    taking_off,
    completed,
    failed,
};

struct FlightStartupSnapshot {
    FlightStartupPhase phase{FlightStartupPhase::disabled};
    std::string detail{"Flight startup disabled"};
    double target_altitude_m{0.0};
    std::size_t attempt{0};
    std::optional<std::uint8_t> failure_result;
};

} // namespace onboard_autonomy::mission
