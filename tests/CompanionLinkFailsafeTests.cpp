#include "TestCases.hpp"

#include "onboard_autonomy/mission/safety/CompanionLinkFailsafe.hpp"

#include <limits>
#include <stdexcept>
#include <string>

namespace {

using onboard_autonomy::mission::ArduPilotGcsFailsafeAction;
using onboard_autonomy::mission::CompanionLinkFailsafe;
using onboard_autonomy::mission::CompanionLinkFailsafePhase;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

CompanionLinkFailsafe configured_policy(const double action = 5.0,
    const double timeout_s = 3.0,
    const double options = 0.0,
    const double gcs_system_id = 1.0) {
    CompanionLinkFailsafe failsafe;
    failsafe.observe_vehicle(true, 1);
    failsafe.on_parameter(1, 1, "FS_GCS_ENABLE", action);
    failsafe.on_parameter(1, 1, "FS_GCS_TIMEOUT", timeout_s);
    failsafe.on_parameter(1, 1, "FS_OPTIONS", options);
    failsafe.on_parameter(1, 1, "SYSID_MYGCS", gcs_system_id);
    return failsafe;
}

void exact_land_policy_is_accepted() {
    const auto snapshot = configured_policy().snapshot();
    require(snapshot.phase == CompanionLinkFailsafePhase::accepted,
        "Always LAND policy must be accepted");
    require(snapshot.action == ArduPilotGcsFailsafeAction::land &&
                snapshot.timeout_s == 3.0 && snapshot.options == 0U &&
                snapshot.configured_gcs_system_id == 1,
        "accepted policy must expose typed ArduPilot values");
}

void unsafe_parameter_combinations_are_rejected() {
    require(configured_policy(0.0).snapshot().phase ==
                CompanionLinkFailsafePhase::rejected,
        "disabled GCS failsafe must be rejected");
    require(configured_policy(5.0, 120.0).snapshot().phase ==
                CompanionLinkFailsafePhase::rejected,
        "excessive companion-loss timeout must be rejected");
    const auto nonfinite =
        configured_policy(5.0, std::numeric_limits<double>::quiet_NaN())
            .snapshot();
    require(nonfinite.phase == CompanionLinkFailsafePhase::rejected &&
                !nonfinite.timeout_s.has_value(),
        "non-finite timeout must be rejected without invalid JSON data");
    require(configured_policy(5.0, 3.0, 2.0).snapshot().phase ==
                CompanionLinkFailsafePhase::rejected,
        "Auto continuation bit must be rejected");
    require(configured_policy(5.0, 3.0, 16.0).snapshot().phase ==
                CompanionLinkFailsafePhase::rejected,
        "pilot-mode continuation bit must be rejected");
    require(configured_policy(5.0, 3.0, 0.0, 255.0).snapshot().phase ==
                CompanionLinkFailsafePhase::rejected,
        "SYSID_MYGCS must identify the companion heartbeat");
}

void wrong_vehicle_values_are_ignored_and_disconnect_resets() {
    CompanionLinkFailsafe failsafe;
    failsafe.observe_vehicle(true, 1);
    failsafe.on_parameter(2, 1, "FS_GCS_ENABLE", 5.0);
    require(failsafe.snapshot().parameters_received == 0,
        "parameters from another vehicle must be ignored");
    failsafe.on_parameter(1, 191, "FS_GCS_ENABLE", 5.0);
    require(failsafe.snapshot().parameters_received == 0,
        "parameters from a non-autopilot component must be ignored");

    failsafe = configured_policy();
    failsafe.observe_vehicle(false, std::nullopt);
    require(failsafe.snapshot().phase ==
                    CompanionLinkFailsafePhase::waiting_for_vehicle &&
                failsafe.snapshot().parameters_received == 0,
        "link loss must invalidate cached failsafe evidence");
}

} // namespace

void run_companion_link_failsafe_tests() {
    exact_land_policy_is_accepted();
    unsafe_parameter_combinations_are_rejected();
    wrong_vehicle_values_are_ignored_and_disconnect_resets();
}
