#include "onboard_autonomy/mission/autonomy/AutonomyRuntime.hpp"

#include "onboard_autonomy/mission/autonomy/WorldState.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace onboard_autonomy::mission {
namespace {

constexpr double kMaximumRelativeYawDegrees = 180.0;

} // namespace

AutonomyRuntime::AutonomyRuntime(const AutonomyRuntimeConfig& config)
    : config_(config) {
    if (config_.target_loss_land_after <= std::chrono::milliseconds::zero()) {
        throw std::invalid_argument(
            "target-loss LAND timeout must be positive");
    }
    if (!std::isfinite(config_.terminal_descent_altitude_m) ||
        config_.terminal_descent_altitude_m <= 0.0 ||
        !std::isfinite(config_.terminal_alignment_radius_m) ||
        config_.terminal_alignment_radius_m <= 0.0 ||
        config_.terminal_alignment_duration <=
            std::chrono::milliseconds::zero()) {
        throw std::invalid_argument(
            "terminal-descent thresholds must be positive");
    }
    if (config_.yaw_command_interval <= std::chrono::milliseconds::zero() ||
        !std::isfinite(config_.yaw_deadband_ratio) ||
        config_.yaw_deadband_ratio < 0.0 || config_.yaw_deadband_ratio >= 1.0 ||
        !std::isfinite(config_.maximum_yaw_step_degrees) ||
        config_.maximum_yaw_step_degrees <= 0.0 ||
        config_.maximum_yaw_step_degrees > kMaximumRelativeYawDegrees ||
        !std::isfinite(config_.yaw_speed_degrees_per_second) ||
        config_.yaw_speed_degrees_per_second <= 0.0 ||
        !std::isfinite(config_.forward_camera_horizontal_fov_radians) ||
        config_.forward_camera_horizontal_fov_radians <= 0.0) {
        throw std::invalid_argument("invalid aerial yaw guidance thresholds");
    }
    if (config_.enabled && config_.start_automatically) {
        restart();
    } else if (config_.enabled) {
        cancel("Waiting for operator mission selection");
    }
}

std::vector<FlightActionRequest> AutonomyRuntime::update(
    const mission::VehicleSnapshot& vehicle,
    const FlightStartupSnapshot& startup,
    const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
    const mission::TimePoint now,
    std::optional<mission::BodyFramePosition> landing_target,
    std::optional<AerialTargetTrackSnapshot> aerial_target) {
    if (phase_ == AutonomyRuntimePhase::disabled ||
        phase_ == AutonomyRuntimePhase::idle ||
        phase_ == AutonomyRuntimePhase::completed ||
        phase_ == AutonomyRuntimePhase::failed) {
        return {};
    }
    if (phase_ == AutonomyRuntimePhase::returning_to_launch) {
        return update_return_to_launch(vehicle);
    }
    if (!prepare_active_runtime(startup, companion_link_failsafe, now) ||
        !validate_runtime_context(vehicle, companion_link_failsafe)) {
        return {};
    }

    if (config_.mode == AutonomyRuntimeMode::aerial_observation) {
        return update_aerial_observation(vehicle, now, aerial_target);
    }

    if (!continue_landing_update(vehicle, now, landing_target)) {
        return {};
    }

    auto world = make_world_state(vehicle, landing_target, now);
    const auto desired = decision_engine_.decide(world);
    const auto supervised = safety_supervisor_.supervise(world, desired, now);
    motion_safety_status_ = supervised.status;

    if (!supervised.approved.has_value()) {
        return handle_missing_target(vehicle, now);
    }
    return handle_approved_motion(*supervised.approved, now);
}

std::vector<FlightActionRequest> AutonomyRuntime::update_aerial_observation(
    const mission::VehicleSnapshot& vehicle,
    const mission::TimePoint now,
    const std::optional<AerialTargetTrackSnapshot>& aerial_target) {
    if (!vehicle.armed) {
        fail("Vehicle disarmed during aerial observation");
        return {};
    }
    motion_safety_status_ = MotionSafetyStatus::no_intent;
    if (!aerial_target.has_value() ||
        aerial_target->phase == AerialTargetTrackPhase::searching) {
        detail_ = "AIRPLANE SEARCHING | GUIDED HOLD";
        return {};
    }
    if (aerial_target->phase == AerialTargetTrackPhase::acquiring) {
        detail_ = "AIRPLANE ACQUIRING " +
                  std::to_string(aerial_target->consecutive_observations) +
                  "/" + std::to_string(aerial_target->required_observations) +
                  " | GUIDED HOLD";
        return {};
    }
    if (!aerial_target->horizontal_error.has_value()) {
        detail_ = "AIRPLANE LOCK INVALID | GUIDED HOLD";
        return {};
    }

    const auto horizontal_error = *aerial_target->horizontal_error;
    if (!std::isfinite(horizontal_error) ||
        std::abs(horizontal_error) <= config_.yaw_deadband_ratio) {
        detail_ = "AIRPLANE LOCKED | CENTERED | GUIDED HOLD";
        return {};
    }
    if (now < next_yaw_command_ || !vehicle_system_id_.has_value()) {
        detail_ = "AIRPLANE LOCKED | YAW ALIGNING | GUIDED HOLD";
        return {};
    }

    constexpr double kRadiansToDegrees = 180.0 / std::numbers::pi;
    const auto half_fov_degrees =
        config_.forward_camera_horizontal_fov_radians * kRadiansToDegrees / 2.0;
    const auto yaw_degrees = std::clamp(horizontal_error * half_fov_degrees,
        -config_.maximum_yaw_step_degrees,
        config_.maximum_yaw_step_degrees);
    next_yaw_command_ = now + config_.yaw_command_interval;
    motion_safety_status_ = MotionSafetyStatus::allowed;

    std::ostringstream detail;
    detail << "AIRPLANE LOCKED | YAW "
           << (yaw_degrees >= 0.0 ? "RIGHT " : "LEFT ") << std::fixed
           << std::setprecision(1) << std::abs(yaw_degrees)
           << " DEG | GUIDED HOLD";
    detail_ = detail.str();
    return {{
        .action = FlightAction::condition_yaw,
        .vehicle_system_id = *vehicle_system_id_,
        .confirmation = 0,
        .altitude_m = 0.0,
        .x_m = 0.0,
        .y_m = 0.0,
        .z_m = 0.0,
        .yaw_degrees = yaw_degrees,
        .yaw_speed_degrees_per_second = config_.yaw_speed_degrees_per_second,
        .time_usec = 0,
    }};
}

bool AutonomyRuntime::prepare_active_runtime(
    const FlightStartupSnapshot& startup,
    const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
    const mission::TimePoint now) {
    if (phase_ != AutonomyRuntimePhase::waiting_for_startup) {
        return true;
    }
    if (startup.phase == FlightStartupPhase::failed) {
        fail("Flight startup failed: " + startup.detail);
        return false;
    }
    if (startup.phase != FlightStartupPhase::completed) {
        detail_ = "Waiting for verified flight startup";
        return false;
    }
    if (!companion_link_failsafe.accepted()) {
        fail("Companion-link failsafe is not valid: " +
             companion_link_failsafe.detail);
        return false;
    }

    phase_ = AutonomyRuntimePhase::active;
    detail_ = config_.mode == AutonomyRuntimeMode::aerial_observation
                  ? "Aerial observation active; holding after takeoff"
                  : "Autonomy active; waiting for vision target";
    target_missing_since_ = now;
    next_landing_target_ = now;
    return true;
}

bool AutonomyRuntime::validate_runtime_context(
    const mission::VehicleSnapshot& vehicle,
    const CompanionLinkFailsafeSnapshot& companion_link_failsafe) {
    if (!vehicle.connected || !vehicle.system_id.has_value()) {
        fail("Flight-controller heartbeat was lost during autonomy");
        return false;
    }
    if (!companion_link_failsafe.accepted()) {
        fail("Companion-link failsafe is no longer valid: " +
             companion_link_failsafe.detail);
        return false;
    }
    vehicle_system_id_ = vehicle.system_id;
    return true;
}

bool AutonomyRuntime::continue_landing_update(
    const mission::VehicleSnapshot& vehicle,
    const mission::TimePoint now,
    const std::optional<mission::BodyFramePosition>& landing_target) {
    if (phase_ == AutonomyRuntimePhase::landing && !vehicle.armed) {
        phase_ = AutonomyRuntimePhase::completed;
        detail_ = "Landing complete; vehicle disarmed";
        vision_landing_target_active_ = false;
        return false;
    }
    if (phase_ == AutonomyRuntimePhase::active && !vehicle.armed) {
        fail("Vehicle disarmed while autonomy was active");
        return false;
    }
    if (phase_ == AutonomyRuntimePhase::landing && now >= landing_deadline_) {
        fail("Vehicle did not complete landing");
        return false;
    }
    if (phase_ == AutonomyRuntimePhase::landing && terminal_descent_active_) {
        vision_landing_target_active_ = false;
        detail_ = "TARGET OUT OF VIEW - TERMINAL DESCENT CONTINUES";
        return false;
    }

    update_terminal_alignment(vehicle, now, landing_target);
    if (phase_ == AutonomyRuntimePhase::landing &&
        vehicle.relative_altitude_m.has_value() &&
        *vehicle.relative_altitude_m <= kTargetStopAltitudeM) {
        vision_landing_target_active_ = false;
        detail_ = "Touchdown detected; waiting for ArduPilot auto-disarm";
        return false;
    }
    return true;
}

void AutonomyRuntime::update_terminal_alignment(
    const mission::VehicleSnapshot& vehicle,
    const mission::TimePoint now,
    const std::optional<mission::BodyFramePosition>& landing_target) {
    const bool can_measure_alignment =
        phase_ == AutonomyRuntimePhase::landing &&
        vehicle.relative_altitude_m.has_value() &&
        *vehicle.relative_altitude_m <= config_.terminal_descent_altitude_m &&
        landing_target.has_value();
    if (!can_measure_alignment) {
        if (landing_target.has_value()) {
            terminal_alignment_since_.reset();
            terminal_alignment_confirmed_ = false;
        }
        return;
    }

    const double lateral_error_m =
        std::hypot(landing_target->forward_m, landing_target->right_m);
    if (lateral_error_m > config_.terminal_alignment_radius_m) {
        terminal_alignment_since_.reset();
        terminal_alignment_confirmed_ = false;
        return;
    }
    if (!terminal_alignment_since_.has_value()) {
        terminal_alignment_since_ = now;
    }
    terminal_alignment_confirmed_ =
        now - *terminal_alignment_since_ >= config_.terminal_alignment_duration;
}

std::vector<FlightActionRequest> AutonomyRuntime::handle_missing_target(
    const mission::VehicleSnapshot& vehicle,
    const mission::TimePoint now) {
    vision_landing_target_active_ = false;
    land_command_after_.reset();
    next_landing_target_ = now;
    if (!target_missing_since_.has_value()) {
        target_missing_since_ = now;
    }

    if (phase_ == AutonomyRuntimePhase::landing) {
        if (vehicle.relative_altitude_m.has_value() &&
            *vehicle.relative_altitude_m <=
                config_.terminal_descent_altitude_m &&
            terminal_alignment_confirmed_) {
            terminal_descent_active_ = true;
            detail_ = "TARGET OUT OF VIEW - TERMINAL DESCENT CONTINUES";
        } else {
            terminal_alignment_since_.reset();
            terminal_alignment_confirmed_ = false;
            detail_ = "Landing without a fresh vision target";
        }
        return {};
    }

    if (now - *target_missing_since_ < config_.target_loss_land_after) {
        detail_ = "Waiting for a fresh confirmed vision target";
        return {};
    }
    detail_ = "Vision unavailable; requesting fallback LAND";
    if (auto land = update_land_command(now)) {
        return {*land};
    }
    return {};
}

std::vector<FlightActionRequest> AutonomyRuntime::handle_approved_motion(
    const DesiredMotion& intent,
    const mission::TimePoint now) {
    std::vector<FlightActionRequest> actions;
    target_missing_since_.reset();
    if (!land_command_after_.has_value() &&
        phase_ == AutonomyRuntimePhase::active) {
        land_command_after_ = now + kLandingTargetWarmup;
        next_landing_target_ = now;
    }

    if (now >= next_landing_target_) {
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch());
        actions.push_back({
            .action = FlightAction::landing_target,
            .vehicle_system_id = intent.vehicle_system_id,
            .x_m = intent.landing_target.forward_m,
            .y_m = intent.landing_target.right_m,
            .z_m = intent.landing_target.down_m,
            .time_usec = static_cast<std::uint64_t>(elapsed.count()),
        });
        next_landing_target_ = now + kLandingTargetInterval;
    }
    if (phase_ == AutonomyRuntimePhase::active &&
        land_command_after_.has_value() && now >= *land_command_after_) {
        if (auto land = update_land_command(now)) {
            actions.push_back(*land);
        }
    }

    std::ostringstream target;
    target << "Vision target F/R/D " << std::fixed << std::setprecision(1)
           << intent.landing_target.forward_m << "/"
           << intent.landing_target.right_m << "/"
           << intent.landing_target.down_m << " m";
    detail_ = target.str();
    return actions;
}

void AutonomyRuntime::on_action_sent(const FlightActionRequest& request,
    const bool sent,
    const mission::TimePoint) {
    if (request.action == FlightAction::return_to_launch) {
        if (!sent && phase_ == AutonomyRuntimePhase::returning_to_launch) {
            fail("Failed to send RTL");
        }
        return;
    }
    if (request.action == FlightAction::landing_target) {
        vision_landing_target_active_ = sent;
        if (!sent) {
            detail_ = "Failed to send vision LANDING_TARGET";
        }
        return;
    }
    if (request.action != FlightAction::land || sent) {
        return;
    }

    awaiting_land_ack_ = false;
    if (land_attempt_ >= kMaximumLandAttempts) {
        fail("Failed to send fallback LAND");
    }
}

void AutonomyRuntime::on_command_ack(const FlightAction action,
    const FlightCommandAckOutcome outcome,
    const std::uint8_t raw_result,
    const std::uint8_t source_system,
    const mission::TimePoint now) {
    if (action == FlightAction::return_to_launch &&
        phase_ == AutonomyRuntimePhase::returning_to_launch &&
        vehicle_system_id_.has_value() &&
        source_system == *vehicle_system_id_) {
        if (outcome == FlightCommandAckOutcome::rejected) {
            failure_result_ = raw_result;
            fail("RTL was rejected with MAV_RESULT " +
                 std::to_string(raw_result));
        } else {
            detail_ = outcome == FlightCommandAckOutcome::accepted
                          ? "RTL accepted; monitoring return"
                          : "RTL is in progress";
        }
        return;
    }
    if (action != FlightAction::land || !awaiting_land_ack_ ||
        !vehicle_system_id_.has_value() ||
        source_system != *vehicle_system_id_) {
        return;
    }

    switch (outcome) {
    case FlightCommandAckOutcome::accepted:
        awaiting_land_ack_ = false;
        phase_ = AutonomyRuntimePhase::landing;
        landing_deadline_ = now + kLandingTimeout;
        detail_ = "LAND accepted; monitoring descent";
        break;
    case FlightCommandAckOutcome::in_progress:
        acknowledgement_deadline_ = now + kAcknowledgementTimeout;
        detail_ = "LAND is in progress";
        break;
    case FlightCommandAckOutcome::rejected:
        failure_result_ = raw_result;
        fail("LAND was rejected with MAV_RESULT " + std::to_string(raw_result));
        break;
    }
}

void AutonomyRuntime::restart() {
    if (!config_.enabled) {
        phase_ = AutonomyRuntimePhase::disabled;
        detail_ = "Autonomy runtime disabled";
    } else {
        phase_ = AutonomyRuntimePhase::waiting_for_startup;
        detail_ = config_.mode == AutonomyRuntimeMode::aerial_observation
                      ? "Preparing takeoff for aerial observation"
                      : "Waiting for verified flight startup";
    }
    reset_runtime_state();
}

void AutonomyRuntime::restart(const AutonomyRuntimeMode mode) {
    config_.mode = mode;
    restart();
}

void AutonomyRuntime::cancel(std::string detail) {
    restart();
    if (config_.enabled) {
        phase_ = AutonomyRuntimePhase::idle;
        detail_ = std::move(detail);
    }
}

void AutonomyRuntime::begin_return_to_launch(
    const std::uint8_t vehicle_system_id) {
    reset_runtime_state();
    phase_ = AutonomyRuntimePhase::returning_to_launch;
    detail_ = "RTL requested; waiting for ArduPilot";
    vehicle_system_id_ = vehicle_system_id;
}

void AutonomyRuntime::reset_runtime_state() {
    motion_safety_status_ = MotionSafetyStatus::no_intent;
    vehicle_system_id_.reset();
    land_command_after_.reset();
    target_missing_since_.reset();
    terminal_alignment_since_.reset();
    next_landing_target_ = {};
    next_yaw_command_ = {};
    acknowledgement_deadline_ = {};
    landing_deadline_ = {};
    land_attempt_ = 0;
    awaiting_land_ack_ = false;
    vision_landing_target_active_ = false;
    terminal_alignment_confirmed_ = false;
    terminal_descent_active_ = false;
    failure_result_.reset();
}

std::vector<FlightActionRequest> AutonomyRuntime::update_return_to_launch(
    const mission::VehicleSnapshot& vehicle) {
    if (!vehicle.connected) {
        fail("Flight-controller heartbeat was lost during RTL");
    } else if (!vehicle.armed) {
        phase_ = AutonomyRuntimePhase::completed;
        detail_ = "RTL complete; vehicle disarmed";
    }
    return {};
}

AutonomyRuntimeSnapshot AutonomyRuntime::snapshot() const {
    return {
        .phase = phase_,
        .detail = detail_,
        .motion_safety_status = motion_safety_status_,
        .vision_landing_target_active = vision_landing_target_active_,
        .terminal_descent_active = terminal_descent_active_,
        .land_attempt = land_attempt_,
        .failure_result = failure_result_,
    };
}

std::optional<FlightActionRequest> AutonomyRuntime::update_land_command(
    const mission::TimePoint now) {
    if (phase_ != AutonomyRuntimePhase::active) {
        return std::nullopt;
    }
    if (awaiting_land_ack_) {
        if (now < acknowledgement_deadline_) {
            return std::nullopt;
        }
        awaiting_land_ack_ = false;
    }
    if (land_attempt_ >= kMaximumLandAttempts) {
        fail("No COMMAND_ACK for LAND after 3 attempts");
        return std::nullopt;
    }
    if (!vehicle_system_id_.has_value()) {
        fail("Cannot send LAND without a vehicle system ID");
        return std::nullopt;
    }

    const auto confirmation = static_cast<std::uint8_t>(land_attempt_);
    ++land_attempt_;
    awaiting_land_ack_ = true;
    acknowledgement_deadline_ = now + kAcknowledgementTimeout;
    return FlightActionRequest{
        .action = FlightAction::land,
        .vehicle_system_id = vehicle_system_id_.value(),
        .confirmation = confirmation,
    };
}

void AutonomyRuntime::fail(std::string detail) {
    phase_ = AutonomyRuntimePhase::failed;
    detail_ = std::move(detail);
    awaiting_land_ack_ = false;
    vision_landing_target_active_ = false;
}

} // namespace onboard_autonomy::mission
