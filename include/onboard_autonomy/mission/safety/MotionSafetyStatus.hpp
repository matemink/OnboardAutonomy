#pragma once

namespace onboard_autonomy::mission {

enum class MotionSafetyStatus {
    allowed,
    no_intent,
    flight_controller_disconnected,
};

} // namespace onboard_autonomy::mission
