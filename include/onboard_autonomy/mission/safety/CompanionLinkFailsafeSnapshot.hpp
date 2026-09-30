#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace onboard_autonomy::mission {

enum class CompanionLinkFailsafePhase {
    waiting_for_vehicle,
    reading_parameters,
    accepted,
    rejected,
};

enum class ArduPilotGcsFailsafeAction : std::uint8_t {
    disabled = 0,
    rtl = 1,
    removed_continue_mission = 2,
    smart_rtl_or_rtl = 3,
    smart_rtl_or_land = 4,
    land = 5,
    auto_land_start_or_rtl = 6,
    brake_or_land = 7,
};

struct CompanionLinkFailsafeSnapshot {
    CompanionLinkFailsafePhase phase{
        CompanionLinkFailsafePhase::waiting_for_vehicle};
    std::string detail{"Waiting for the flight-controller heartbeat"};
    std::optional<std::uint8_t> heartbeat_system_id;
    std::optional<std::uint8_t> configured_gcs_system_id;
    std::optional<ArduPilotGcsFailsafeAction> action;
    std::optional<double> timeout_s;
    std::optional<std::uint32_t> options;
    std::size_t parameters_received{0};
    std::size_t parameters_required{4};

    [[nodiscard]] bool accepted() const;
};

[[nodiscard]] std::string_view companion_link_failsafe_phase_name(
    CompanionLinkFailsafePhase phase);

[[nodiscard]] std::string_view ardupilot_gcs_failsafe_action_name(
    ArduPilotGcsFailsafeAction action);

} // namespace onboard_autonomy::mission
