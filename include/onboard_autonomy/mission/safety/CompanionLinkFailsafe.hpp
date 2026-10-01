#pragma once

#include "onboard_autonomy/mission/safety/CompanionLinkFailsafeSnapshot.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace onboard_autonomy::mission {

class CompanionLinkFailsafe {
  public:
    static constexpr std::array<std::string_view, 4> parameter_names{
        "FS_GCS_ENABLE",
        "FS_GCS_TIMEOUT",
        "FS_OPTIONS",
        "SYSID_MYGCS",
    };

    void observe_vehicle(bool connected, std::optional<std::uint8_t> system_id);

    void on_parameter(std::uint8_t source_system,
        std::uint8_t source_component,
        std::string_view id,
        double value);

    [[nodiscard]] CompanionLinkFailsafeSnapshot snapshot() const;

  private:
    void reset(std::optional<std::uint8_t> system_id);
    void validate();

    CompanionLinkFailsafeSnapshot snapshot_;
    std::optional<double> raw_action_;
    std::optional<double> raw_timeout_s_;
    std::optional<double> raw_options_;
    std::optional<double> raw_gcs_system_id_;

    static constexpr std::uint8_t kAutopilotComponentId = 1;
};

} // namespace onboard_autonomy::mission
