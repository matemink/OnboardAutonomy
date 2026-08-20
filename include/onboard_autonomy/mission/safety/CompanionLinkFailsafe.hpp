#pragma once

#include <array>
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

[[nodiscard]] std::string_view companion_link_failsafe_phase_name(
    CompanionLinkFailsafePhase phase);

[[nodiscard]] std::string_view ardupilot_gcs_failsafe_action_name(
    ArduPilotGcsFailsafeAction action);

} // namespace onboard_autonomy::mission
