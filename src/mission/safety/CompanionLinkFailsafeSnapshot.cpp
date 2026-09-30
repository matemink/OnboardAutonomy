#include "onboard_autonomy/mission/safety/CompanionLinkFailsafeSnapshot.hpp"

namespace onboard_autonomy::mission {

bool CompanionLinkFailsafeSnapshot::accepted() const {
    return phase == CompanionLinkFailsafePhase::accepted;
}

std::string_view companion_link_failsafe_phase_name(
    const CompanionLinkFailsafePhase phase) {
    switch (phase) {
    case CompanionLinkFailsafePhase::waiting_for_vehicle:
        return "waiting_for_vehicle";
    case CompanionLinkFailsafePhase::reading_parameters:
        return "reading_parameters";
    case CompanionLinkFailsafePhase::accepted:
        return "accepted";
    case CompanionLinkFailsafePhase::rejected:
        return "rejected";
    }
    return "rejected";
}

std::string_view ardupilot_gcs_failsafe_action_name(
    const ArduPilotGcsFailsafeAction action) {
    switch (action) {
    case ArduPilotGcsFailsafeAction::disabled:
        return "disabled";
    case ArduPilotGcsFailsafeAction::rtl:
        return "rtl";
    case ArduPilotGcsFailsafeAction::removed_continue_mission:
        return "removed_continue_mission";
    case ArduPilotGcsFailsafeAction::smart_rtl_or_rtl:
        return "smart_rtl_or_rtl";
    case ArduPilotGcsFailsafeAction::smart_rtl_or_land:
        return "smart_rtl_or_land";
    case ArduPilotGcsFailsafeAction::land:
        return "land";
    case ArduPilotGcsFailsafeAction::auto_land_start_or_rtl:
        return "auto_land_start_or_rtl";
    case ArduPilotGcsFailsafeAction::brake_or_land:
        return "brake_or_land";
    }
    return "unsupported";
}

} // namespace onboard_autonomy::mission
