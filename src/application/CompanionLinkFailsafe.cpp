#include "onboard_autonomy/application/CompanionLinkFailsafe.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace onboard_autonomy::application {
namespace {

constexpr std::uint32_t kGcsFailsafeContinuationMask = (1U << 1U) | (1U << 4U);
constexpr double kMinimumTimeoutS = 2.0;
constexpr double kMaximumTimeoutS = 10.0;

template <typename T>
std::optional<T> exact_integer(const std::optional<double> value,
    const double maximum) {
    if (!value.has_value() || !std::isfinite(*value) || *value < 0.0 ||
        *value > maximum || std::trunc(*value) != *value) {
        return std::nullopt;
    }
    return static_cast<T>(*value);
}

std::string timeout_detail(const double timeout_s) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(1) << timeout_s;
    return output.str();
}

} // namespace

bool CompanionLinkFailsafeSnapshot::accepted() const {
    return phase == CompanionLinkFailsafePhase::accepted;
}

void CompanionLinkFailsafe::observe_vehicle(const bool connected,
    const std::optional<std::uint8_t> system_id) {
    if (!connected || !system_id.has_value()) {
        if (snapshot_.heartbeat_system_id.has_value()) {
            reset(std::nullopt);
        }
        return;
    }

    if (snapshot_.heartbeat_system_id != system_id) {
        reset(system_id);
    }
}

void CompanionLinkFailsafe::on_parameter(const std::uint8_t source_system,
    const std::uint8_t source_component,
    const std::string_view id,
    const double value) {
    if (!snapshot_.heartbeat_system_id.has_value() ||
        source_system != *snapshot_.heartbeat_system_id ||
        source_component != kAutopilotComponentId) {
        return;
    }

    if (id == "FS_GCS_ENABLE") {
        raw_action_ = value;
    } else if (id == "FS_GCS_TIMEOUT") {
        raw_timeout_s_ = value;
    } else if (id == "FS_OPTIONS") {
        raw_options_ = value;
    } else if (id == "SYSID_MYGCS") {
        raw_gcs_system_id_ = value;
    } else {
        return;
    }

    validate();
}

CompanionLinkFailsafeSnapshot CompanionLinkFailsafe::snapshot() const {
    return snapshot_;
}

void CompanionLinkFailsafe::reset(const std::optional<std::uint8_t> system_id) {
    snapshot_ = {};
    snapshot_.heartbeat_system_id = system_id;
    if (system_id.has_value()) {
        snapshot_.phase = CompanionLinkFailsafePhase::reading_parameters;
        snapshot_.detail =
            "Reading ArduPilot companion-link failsafe parameters";
    }
    raw_action_.reset();
    raw_timeout_s_.reset();
    raw_options_.reset();
    raw_gcs_system_id_.reset();
}

void CompanionLinkFailsafe::validate() {
    snapshot_.parameters_received =
        static_cast<std::size_t>(raw_action_.has_value()) +
        static_cast<std::size_t>(raw_timeout_s_.has_value()) +
        static_cast<std::size_t>(raw_options_.has_value()) +
        static_cast<std::size_t>(raw_gcs_system_id_.has_value());

    if (snapshot_.parameters_received < snapshot_.parameters_required) {
        snapshot_.phase = CompanionLinkFailsafePhase::reading_parameters;
        snapshot_.detail = "Reading ArduPilot failsafe parameters (" +
                           std::to_string(snapshot_.parameters_received) + "/" +
                           std::to_string(snapshot_.parameters_required) + ")";
        return;
    }

    const auto action_value = exact_integer<std::uint8_t>(raw_action_,
        static_cast<double>(std::numeric_limits<std::uint8_t>::max()));
    const auto options_value = exact_integer<std::uint32_t>(raw_options_,
        static_cast<double>(std::numeric_limits<std::uint32_t>::max()));
    const auto gcs_system_id = exact_integer<std::uint8_t>(raw_gcs_system_id_,
        static_cast<double>(std::numeric_limits<std::uint8_t>::max()));

    snapshot_.action =
        action_value.has_value() &&
                *action_value <= static_cast<std::uint8_t>(
                                     ArduPilotGcsFailsafeAction::brake_or_land)
            ? std::optional{static_cast<ArduPilotGcsFailsafeAction>(
                  *action_value)}
            : std::nullopt;
    snapshot_.timeout_s =
        raw_timeout_s_.has_value() && std::isfinite(*raw_timeout_s_)
            ? raw_timeout_s_
            : std::nullopt;
    snapshot_.options = options_value;
    snapshot_.configured_gcs_system_id = gcs_system_id;
    snapshot_.phase = CompanionLinkFailsafePhase::rejected;

    if (!snapshot_.action.has_value()) {
        snapshot_.detail = "FS_GCS_ENABLE is unsupported";
        return;
    }
    if (*snapshot_.action != ArduPilotGcsFailsafeAction::land) {
        snapshot_.detail = "FS_GCS_ENABLE must be 5 (Always LAND)";
        return;
    }
    if (!raw_timeout_s_.has_value() || !std::isfinite(*raw_timeout_s_) ||
        *raw_timeout_s_ < kMinimumTimeoutS ||
        *raw_timeout_s_ > kMaximumTimeoutS) {
        snapshot_.detail = "FS_GCS_TIMEOUT must be between 2 and 10 seconds";
        return;
    }
    if (!options_value.has_value()) {
        snapshot_.detail = "FS_OPTIONS is not a valid bitmask";
        return;
    }
    if ((*options_value & kGcsFailsafeContinuationMask) != 0U) {
        snapshot_.detail = "FS_OPTIONS must not bypass GCS failsafe in Auto or "
                           "pilot modes";
        return;
    }
    if (!gcs_system_id.has_value() || *gcs_system_id == 0U) {
        snapshot_.detail = "SYSID_MYGCS is invalid";
        return;
    }
    if (!snapshot_.heartbeat_system_id.has_value() ||
        *gcs_system_id != *snapshot_.heartbeat_system_id) {
        snapshot_.detail =
            "SYSID_MYGCS must match companion heartbeat system id " +
            (snapshot_.heartbeat_system_id.has_value()
                    ? std::to_string(*snapshot_.heartbeat_system_id)
                    : std::string{"unknown"});
        return;
    }

    snapshot_.phase = CompanionLinkFailsafePhase::accepted;
    snapshot_.detail = "ArduPilot will LAND after " +
                       timeout_detail(*raw_timeout_s_) +
                       " s without the companion heartbeat";
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

} // namespace onboard_autonomy::application
