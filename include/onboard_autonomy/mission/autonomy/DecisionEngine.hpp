#pragma once

#include "onboard_autonomy/mission/autonomy/WorldState.hpp"

#include <chrono>
#include <optional>

namespace onboard_autonomy::mission {

enum class DesiredMotionType {
    precision_land,
};

struct DesiredMotion {
    DesiredMotionType type{DesiredMotionType::precision_land};
    std::uint8_t vehicle_system_id{0};
    mission::BodyFramePosition landing_target;
    mission::TimePoint created_at;
    mission::TimePoint valid_until;
};

class DecisionEngine {
  public:
    [[nodiscard]] std::optional<DesiredMotion> decide(
        const WorldState& world) const;

  private:
    static constexpr auto kIntentLifetime = std::chrono::milliseconds(250);
};

} // namespace onboard_autonomy::mission
