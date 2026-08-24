#include "onboard_autonomy/mission/flight/FlightStartupController.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace onboard_autonomy::mission {
namespace {

constexpr std::uint8_t kArduPilotAutopilotType = 3;
constexpr std::uint32_t kCopterGuidedMode = 4;
constexpr std::uint8_t kMavTypeQuadrotor = 2;
constexpr std::uint8_t kMavTypeCoaxial = 3;
constexpr std::uint8_t kMavTypeHelicopter = 4;
constexpr std::uint8_t kMavTypeHexarotor = 13;
constexpr std::uint8_t kMavTypeOctorotor = 14;
constexpr std::uint8_t kMavTypeTricopter = 15;
constexpr std::size_t kMaximumActionAttempts = 3;
constexpr int kMaximumPhaseTransitionsPerUpdate = 8;
constexpr auto kAcknowledgementTimeout = std::chrono::seconds(2);
constexpr auto kReadinessTimeout = std::chrono::seconds(90);
constexpr auto kModeChangeTimeout = std::chrono::seconds(10);
constexpr auto kArmingTimeout = std::chrono::seconds(10);
constexpr auto kTakeoffTimeout = std::chrono::seconds(45);
constexpr double kAltitudeToleranceM = 0.35;

bool is_multicopter(const std::optional<std::uint8_t> vehicle_type) {
    if (!vehicle_type.has_value()) {
        return false;
    }

    switch (*vehicle_type) {
    case kMavTypeQuadrotor:
    case kMavTypeCoaxial:
    case kMavTypeHelicopter:
    case kMavTypeHexarotor:
    case kMavTypeOctorotor:
    case kMavTypeTricopter:
        return true;
    default:
        return false;
    }
}

std::string action_name(const FlightAction action) {
    switch (action) {
    case FlightAction::invalid:
        return "invalid action";
    case FlightAction::set_guided_mode:
        return "GUIDED mode";
    case FlightAction::arm:
        return "arm";
    case FlightAction::takeoff:
        return "takeoff";
    case FlightAction::return_to_launch:
        return "RTL";
    case FlightAction::land:
        return "land";
    case FlightAction::landing_target:
        return "landing target";
    case FlightAction::condition_yaw:
        return "yaw";
    }
    return "unknown action";
}

std::string altitude_detail(const std::optional<double> altitude_m,
    const double target_m) {
    std::ostringstream output;
    if (altitude_m.has_value()) {
        output << std::fixed << std::setprecision(2) << *altitude_m << " m";
    } else {
        output << "waiting for altitude";
    }
    output << " / target " << target_m << " m";
    return output.str();
}

} // namespace

FlightStartupController::FlightStartupController(FlightStartupConfig config)
    : config_(config) {
    if (!std::isfinite(config_.takeoff_altitude_m) ||
        config_.takeoff_altitude_m <= 0.0) {
        throw std::invalid_argument(
            "takeoff altitude must be finite and positive");
    }
    if (config_.enabled && config_.start_automatically) {
        restart();
    } else if (config_.enabled) {
        cancel("Waiting for operator mission selection");
    }
}

std::vector<FlightActionRequest> FlightStartupController::update(
    const mission::VehicleSnapshot& vehicle,
    const bool telemetry_ready,
    const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
    const mission::TimePoint now) {
    std::vector<FlightActionRequest> actions;
    if (phase_ == FlightStartupPhase::disabled ||
        phase_ == FlightStartupPhase::idle ||
        phase_ == FlightStartupPhase::completed ||
        phase_ == FlightStartupPhase::failed) {
        return actions;
    }

    if (!validate_active_context(vehicle, companion_link_failsafe)) {
        return actions;
    }

    for (int transition = 0; transition < kMaximumPhaseTransitionsPerUpdate;
         ++transition) {
        if (update_current_phase(vehicle,
                telemetry_ready,
                companion_link_failsafe,
                now,
                actions) == PhaseUpdate::stop) {
            return actions;
        }
    }

    fail("Flight startup made too many immediate transitions");
    return actions;
}

bool FlightStartupController::validate_active_context(
    const mission::VehicleSnapshot& vehicle,
    const CompanionLinkFailsafeSnapshot& failsafe) {
    if (phase_ != FlightStartupPhase::waiting_for_vehicle &&
        !vehicle.connected) {
        fail("Flight-controller heartbeat was lost during startup");
        return false;
    }
    if (phase_ != FlightStartupPhase::waiting_for_vehicle &&
        phase_ != FlightStartupPhase::waiting_for_readiness &&
        !failsafe.accepted()) {
        fail("Companion-link failsafe is no longer valid: " + failsafe.detail);
        return false;
    }
    return true;
}

FlightStartupController::PhaseUpdate
FlightStartupController::update_current_phase(
    const mission::VehicleSnapshot& vehicle,
    const bool telemetry_ready,
    const CompanionLinkFailsafeSnapshot& failsafe,
    const mission::TimePoint now,
    std::vector<FlightActionRequest>& actions) {
    switch (phase_) {
    case FlightStartupPhase::waiting_for_vehicle:
        return update_waiting_for_vehicle(vehicle, now);
    case FlightStartupPhase::waiting_for_readiness:
        return update_waiting_for_readiness(vehicle,
            telemetry_ready,
            failsafe,
            now);
    case FlightStartupPhase::setting_guided:
        return update_setting_guided(vehicle, now, actions);
    case FlightStartupPhase::arming:
        return update_arming(vehicle, now, actions);
    case FlightStartupPhase::taking_off:
        return update_taking_off(vehicle, now, actions);
    case FlightStartupPhase::disabled:
    case FlightStartupPhase::idle:
    case FlightStartupPhase::completed:
    case FlightStartupPhase::failed:
        return PhaseUpdate::stop;
    }
    return PhaseUpdate::stop;
}

FlightStartupController::PhaseUpdate
FlightStartupController::update_waiting_for_vehicle(
    const mission::VehicleSnapshot& vehicle,
    const mission::TimePoint now) {
    if (!vehicle.connected || !vehicle.system_id.has_value()) {
        return PhaseUpdate::stop;
    }
    if (vehicle.autopilot_type != kArduPilotAutopilotType ||
        !is_multicopter(vehicle.vehicle_type)) {
        fail("Autonomy requires an ArduPilot multicopter");
        return PhaseUpdate::stop;
    }
    vehicle_system_id_ = vehicle.system_id;
    enter_phase(FlightStartupPhase::waiting_for_readiness, now);
    return PhaseUpdate::advance;
}

FlightStartupController::PhaseUpdate
FlightStartupController::update_waiting_for_readiness(
    const mission::VehicleSnapshot& vehicle,
    const bool telemetry_ready,
    const CompanionLinkFailsafeSnapshot& failsafe,
    const mission::TimePoint now) {
    if (now >= phase_deadline_) {
        fail("Vehicle did not become ready within 90 seconds");
        return PhaseUpdate::stop;
    }
    if (telemetry_ready && vehicle.armable && !vehicle.armed &&
        failsafe.accepted() && vehicle.relative_altitude_m.has_value()) {
        enter_phase(FlightStartupPhase::setting_guided, now);
        return PhaseUpdate::advance;
    }
    update_readiness_detail(vehicle, telemetry_ready, failsafe);
    return PhaseUpdate::stop;
}

FlightStartupController::PhaseUpdate
FlightStartupController::update_setting_guided(
    const mission::VehicleSnapshot& vehicle,
    const mission::TimePoint now,
    std::vector<FlightActionRequest>& actions) {
    if (command_accepted_ && vehicle.flight_mode == kCopterGuidedMode) {
        enter_phase(FlightStartupPhase::arming, now);
        return PhaseUpdate::advance;
    }
    if (now >= phase_deadline_) {
        fail("ArduCopter did not enter GUIDED mode");
        return PhaseUpdate::stop;
    }
    if (auto request = update_command(FlightAction::set_guided_mode, now)) {
        actions.push_back(*request);
    }
    return PhaseUpdate::stop;
}

FlightStartupController::PhaseUpdate FlightStartupController::update_arming(
    const mission::VehicleSnapshot& vehicle,
    const mission::TimePoint now,
    std::vector<FlightActionRequest>& actions) {
    if (command_accepted_ && vehicle.armed) {
        enter_phase(FlightStartupPhase::taking_off, now);
        return PhaseUpdate::advance;
    }
    if (now >= phase_deadline_) {
        fail("ArduCopter did not become armed");
        return PhaseUpdate::stop;
    }
    if (auto request = update_command(FlightAction::arm, now)) {
        actions.push_back(*request);
    }
    return PhaseUpdate::stop;
}

FlightStartupController::PhaseUpdate FlightStartupController::update_taking_off(
    const mission::VehicleSnapshot& vehicle,
    const mission::TimePoint now,
    std::vector<FlightActionRequest>& actions) {
    if (!vehicle.armed && command_accepted_) {
        fail("Vehicle disarmed during takeoff");
        return PhaseUpdate::stop;
    }
    const bool altitude_reached =
        command_accepted_ && vehicle.relative_altitude_m.has_value() &&
        vehicle.relative_altitude_m.value() >=
            config_.takeoff_altitude_m - kAltitudeToleranceM;
    if (altitude_reached) {
        phase_ = FlightStartupPhase::completed;
        detail_ = "Takeoff complete; autonomy may start";
        attempt_ = 0;
        awaiting_ack_ = false;
        return PhaseUpdate::stop;
    }
    if (now >= phase_deadline_) {
        fail("Vehicle did not reach takeoff altitude");
        return PhaseUpdate::stop;
    }
    if (command_accepted_) {
        detail_ = "Climbing: " + altitude_detail(vehicle.relative_altitude_m,
                                     config_.takeoff_altitude_m);
    }
    if (auto request = update_command(FlightAction::takeoff, now)) {
        request->altitude_m = config_.takeoff_altitude_m;
        actions.push_back(*request);
    }
    return PhaseUpdate::stop;
}

void FlightStartupController::on_action_sent(const FlightActionRequest& request,
    const bool sent,
    const mission::TimePoint) {
    const auto expected = expected_action();
    if (!expected.has_value() || request.action != *expected || sent) {
        return;
    }

    awaiting_ack_ = false;
    if (attempt_ >= kMaximumActionAttempts) {
        fail("Failed to send " + action_name(request.action));
    }
}

void FlightStartupController::on_command_ack(const FlightAction action,
    const FlightCommandAckOutcome outcome,
    const std::uint8_t raw_result,
    const std::uint8_t source_system,
    const mission::TimePoint now) {
    const auto expected = expected_action();
    if (!expected.has_value() || action != *expected || !awaiting_ack_ ||
        !vehicle_system_id_.has_value() ||
        source_system != *vehicle_system_id_) {
        return;
    }

    switch (outcome) {
    case FlightCommandAckOutcome::accepted:
        awaiting_ack_ = false;
        command_accepted_ = true;
        detail_ = action_name(action) + " accepted; verifying vehicle state";
        break;
    case FlightCommandAckOutcome::in_progress:
        acknowledgement_deadline_ = now + kAcknowledgementTimeout;
        detail_ = action_name(action) + " is in progress";
        break;
    case FlightCommandAckOutcome::rejected:
        failure_result_ = raw_result;
        fail(action_name(action) + " was rejected with MAV_RESULT " +
             std::to_string(raw_result));
        break;
    }
}

void FlightStartupController::restart() {
    if (!config_.enabled) {
        phase_ = FlightStartupPhase::disabled;
        detail_ = "Flight startup disabled";
    } else {
        phase_ = FlightStartupPhase::waiting_for_vehicle;
        detail_ = "Waiting for the flight-controller heartbeat";
    }
    vehicle_system_id_.reset();
    attempt_ = 0;
    awaiting_ack_ = false;
    command_accepted_ = false;
    acknowledgement_deadline_ = {};
    phase_deadline_ = {};
    failure_result_.reset();
}

void FlightStartupController::cancel(std::string detail) {
    restart();
    if (config_.enabled) {
        phase_ = FlightStartupPhase::idle;
        detail_ = std::move(detail);
    }
}

FlightStartupSnapshot FlightStartupController::snapshot() const {
    return {
        .phase = phase_,
        .detail = detail_,
        .target_altitude_m = config_.takeoff_altitude_m,
        .attempt = attempt_,
        .failure_result = failure_result_,
    };
}

void FlightStartupController::enter_phase(const FlightStartupPhase phase,
    const mission::TimePoint now) {
    phase_ = phase;
    attempt_ = 0;
    awaiting_ack_ = false;
    command_accepted_ = false;

    switch (phase_) {
    case FlightStartupPhase::waiting_for_readiness:
        phase_deadline_ = now + kReadinessTimeout;
        detail_ = "Waiting for telemetry and pre-arm readiness";
        break;
    case FlightStartupPhase::setting_guided:
        phase_deadline_ = now + kModeChangeTimeout;
        detail_ = "Preparing GUIDED mode";
        break;
    case FlightStartupPhase::arming:
        phase_deadline_ = now + kArmingTimeout;
        detail_ = "Preparing to arm";
        break;
    case FlightStartupPhase::taking_off:
        phase_deadline_ = now + kTakeoffTimeout;
        detail_ = "Preparing takeoff";
        break;
    case FlightStartupPhase::disabled:
    case FlightStartupPhase::idle:
    case FlightStartupPhase::waiting_for_vehicle:
    case FlightStartupPhase::completed:
    case FlightStartupPhase::failed:
        break;
    }
}

void FlightStartupController::fail(std::string detail) {
    phase_ = FlightStartupPhase::failed;
    detail_ = std::move(detail);
    awaiting_ack_ = false;
    command_accepted_ = false;
}

void FlightStartupController::update_readiness_detail(
    const mission::VehicleSnapshot& vehicle,
    const bool telemetry_ready,
    const CompanionLinkFailsafeSnapshot& companion_link_failsafe) {
    if (!telemetry_ready) {
        detail_ = "Waiting for telemetry stream setup";
    } else if (!companion_link_failsafe.accepted()) {
        detail_ = "Autonomy blocked: " + companion_link_failsafe.detail;
    } else if (vehicle.armed) {
        detail_ = "Waiting for the vehicle to be disarmed";
    } else if (!vehicle.navigation_ready) {
        detail_ = "Waiting for a navigation estimate";
    } else if (!vehicle.battery_ready) {
        detail_ = "Waiting for valid battery data";
    } else if (!vehicle.system_health_ok) {
        detail_ = "Waiting for healthy onboard sensors";
    } else if (!vehicle.armable) {
        detail_ = "Waiting for active PreArm warnings to clear";
    } else if (!vehicle.relative_altitude_m.has_value()) {
        detail_ = "Waiting for relative altitude";
    } else {
        detail_ = "Checking readiness";
    }
}

std::optional<FlightActionRequest> FlightStartupController::update_command(
    const FlightAction action,
    const mission::TimePoint now) {
    if (command_accepted_) {
        return std::nullopt;
    }
    if (awaiting_ack_) {
        if (now < acknowledgement_deadline_) {
            return std::nullopt;
        }
        awaiting_ack_ = false;
    }
    if (attempt_ >= kMaximumActionAttempts) {
        fail("No COMMAND_ACK for " + action_name(action) + " after 3 attempts");
        return std::nullopt;
    }

    const auto confirmation = static_cast<std::uint8_t>(attempt_);
    ++attempt_;
    awaiting_ack_ = true;
    acknowledgement_deadline_ = now + kAcknowledgementTimeout;
    detail_ = "Sending " + action_name(action) + " | attempt " +
              std::to_string(attempt_) + "/3";
    return FlightActionRequest{
        .action = action,
        .vehicle_system_id = vehicle_system_id_.value_or(0U),
        .confirmation = confirmation,
    };
}

std::optional<FlightAction> FlightStartupController::expected_action() const {
    switch (phase_) {
    case FlightStartupPhase::setting_guided:
        return FlightAction::set_guided_mode;
    case FlightStartupPhase::arming:
        return FlightAction::arm;
    case FlightStartupPhase::taking_off:
        return FlightAction::takeoff;
    case FlightStartupPhase::disabled:
    case FlightStartupPhase::idle:
    case FlightStartupPhase::waiting_for_vehicle:
    case FlightStartupPhase::waiting_for_readiness:
    case FlightStartupPhase::completed:
    case FlightStartupPhase::failed:
        return std::nullopt;
    }
    return std::nullopt;
}

} // namespace onboard_autonomy::mission
