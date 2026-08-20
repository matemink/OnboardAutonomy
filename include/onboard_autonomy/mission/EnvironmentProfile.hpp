#pragma once

namespace onboard_autonomy::mission {

struct SimulatedWindProfile {
    double speed_m_s{0.0};
    double direction_from_deg{0.0};
    double turbulence_m_s{0.0};
};

} // namespace onboard_autonomy::mission
