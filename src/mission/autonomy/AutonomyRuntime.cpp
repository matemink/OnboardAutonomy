#include "onboard_autonomy/mission/autonomy/AutonomyRuntime.hpp"

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
constexpr double kNormalizedHorizontalErrorToPercent = 50.0;

} // namespace

AutonomyRuntime::AutonomyRuntime(const AutonomyRuntimeConfig& config)
    : config_(config),
      aerial_yaw_controller_({
          .maximum_yaw_rate_degrees_per_second =
              config.yaw_speed_degrees_per_second,
          .horizontal_deadband_ratio = config.yaw_deadband_ratio,
          .horizontal_fov_radians =
              config.forward_camera_horizontal_fov_radians,
      }) {
    if (config_.yaw_command_interval <= std::chrono::milliseconds::zero() ||
        !std::isfinite(config_.yaw_deadband_ratio) ||
        config_.yaw_deadband_ratio < 0.0 || config_.yaw_deadband_ratio >= 1.0 ||
        !std::isfinite(config_.maximum_yaw_step_degrees) ||
        config_.maximum_yaw_step_degrees <= 0.0 ||
        config_.maximum_yaw_step_degrees > kMaximumRelativeYawDegrees ||
        !std::isfinite(config_.yaw_speed_degrees_per_second) ||
        config_.yaw_speed_degrees_per_second <= 0.0 ||
        !std::isfinite(config_.forward_camera_horizontal_fov_radians) ||
        config_.forward_camera_horizontal_fov_radians <= 0.0 ||
        config_.aerial_link_loss_grace <= std::chrono::milliseconds::zero() ||
        config_.aerial_link_recovery_hold <=
            std::chrono::milliseconds::zero()) {
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
    std::optional<AerialTargetTrackSnapshot> aerial_target) {
    if (phase_ == AutonomyRuntimePhase::disabled ||
        phase_ == AutonomyRuntimePhase::idle ||
        phase_ == AutonomyRuntimePhase::completed ||
        phase_ == AutonomyRuntimePhase::failed) {
        return {};
    }
    if (phase_ == AutonomyRuntimePhase::returning_to_launch) {
        return update_return_to_launch(vehicle, now);
    }
    if (!prepare_active_runtime(startup, companion_link_failsafe)) {
        return {};
    }
    const bool runtime_context_valid =
        validate_runtime_context(vehicle, companion_link_failsafe, now);
    if (const auto yaw_stop = pending_aerial_yaw_stop(vehicle);
        yaw_stop.has_value()) {
        return {*yaw_stop};
    }
    if (!runtime_context_valid) {
        return {};
    }

    return update_aerial_observation(vehicle, now, aerial_target);
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
        aerial_horizontal_error_.reset();
        detail_ = "TARGET SEARCHING | GUIDED HOLD";
        return stop_aerial_yaw();
    }
    aerial_horizontal_error_ = aerial_target->horizontal_error;
    if (aerial_target->phase == AerialTargetTrackPhase::acquiring) {
        std::vector<FlightActionRequest> actions;
        if (aerial_yaw_active_) {
            actions = stop_aerial_yaw();
        }
        const bool has_new_observation =
            aerial_target->horizontal_error.has_value() &&
            std::isfinite(*aerial_target->horizontal_error) &&
            (!last_aerial_observation_count_.has_value() ||
                *last_aerial_observation_count_ !=
                    aerial_target->accepted_observations);
        if (has_new_observation) {
            auto observed_at = now;
            if (aerial_target->observation_age_ms.has_value() &&
                std::isfinite(*aerial_target->observation_age_ms) &&
                *aerial_target->observation_age_ms >= 0.0) {
                observed_at -= std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double, std::milli>{
                        *aerial_target->observation_age_ms});
            }
            aerial_yaw_controller_.observe_while_holding(
                *aerial_target->horizontal_error,
                observed_at);
            last_aerial_observation_count_ =
                aerial_target->accepted_observations;
        }
        detail_ = "TARGET ACQUIRING " +
                  std::to_string(aerial_target->consecutive_observations) +
                  "/" + std::to_string(aerial_target->required_observations) +
                  " | GUIDED HOLD";
        return actions;
    }
    if (!aerial_target->horizontal_error.has_value()) {
        detail_ = "TARGET LOCK INVALID | GUIDED HOLD";
        return stop_aerial_yaw();
    }

    const auto horizontal_error = *aerial_target->horizontal_error;
    if (!std::isfinite(horizontal_error)) {
        detail_ = "TARGET LOCK INVALID | GUIDED HOLD";
        return stop_aerial_yaw();
    }
    const bool has_new_observation =
        !last_aerial_observation_count_.has_value() ||
        *last_aerial_observation_count_ != aerial_target->accepted_observations;
    if (has_new_observation) {
        auto observed_at = now;
        if (aerial_target->observation_age_ms.has_value() &&
            std::isfinite(*aerial_target->observation_age_ms) &&
            *aerial_target->observation_age_ms >= 0.0) {
            observed_at -= std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double, std::milli>{
                    *aerial_target->observation_age_ms});
        }
        constexpr double kRadiansToDegrees = 180.0 / std::numbers::pi;
        const auto actual_yaw_rate_degrees_per_second =
            vehicle.yaw_rate_rad_per_second.value_or(0.0) * kRadiansToDegrees;
        const auto control = aerial_yaw_controller_.update(horizontal_error,
            actual_yaw_rate_degrees_per_second,
            observed_at);
        aerial_proportional_rate_degrees_per_second_ =
            control.proportional_rate_degrees_per_second;
        aerial_feed_forward_rate_degrees_per_second_ =
            control.feed_forward_rate_degrees_per_second;
        current_aerial_yaw_rate_degrees_per_second_ =
            control.yaw_rate_degrees_per_second;
        last_aerial_observation_count_ = aerial_target->accepted_observations;
    }
    if (now < next_yaw_command_ || !vehicle_system_id_.has_value() ||
        !current_aerial_yaw_rate_degrees_per_second_.has_value()) {
        detail_ = "TARGET LOCKED | YAW ALIGNING | GUIDED HOLD";
        return {};
    }

    next_yaw_command_ = now + config_.yaw_command_interval;
    motion_safety_status_ = MotionSafetyStatus::allowed;
    aerial_yaw_active_ = true;

    std::ostringstream detail;
    detail << "TARGET LOCKED | CENTER ERROR " << std::fixed
           << std::setprecision(1)
           << horizontal_error * kNormalizedHorizontalErrorToPercent
           << "% | YAW "
           << (*current_aerial_yaw_rate_degrees_per_second_ < 0.0 ? "LEFT "
                                                                  : "RIGHT ")
           << std::abs(*current_aerial_yaw_rate_degrees_per_second_)
           << " DEG/S";
    detail_ = detail.str();
    return {{
        .action = FlightAction::yaw_rate,
        .vehicle_system_id = *vehicle_system_id_,
        .confirmation = 0,
        .altitude_m = 0.0,
        .x_m = 0.0,
        .y_m = 0.0,
        .z_m = 0.0,
        .yaw_rate_degrees_per_second =
            *current_aerial_yaw_rate_degrees_per_second_,
        .time_usec = 0,
    }};
}

std::vector<FlightActionRequest> AutonomyRuntime::stop_aerial_yaw() {
    const bool should_send_stop =
        aerial_yaw_active_ && vehicle_system_id_.has_value();
    const auto vehicle_system_id = vehicle_system_id_;
    clear_aerial_yaw_guidance();
    aerial_yaw_stop_pending_ = false;
    if (!should_send_stop) {
        return {};
    }
    return {{
        .action = FlightAction::yaw_rate,
        .vehicle_system_id = *vehicle_system_id,
        .confirmation = 0,
        .yaw_rate_degrees_per_second = 0.0,
    }};
}

std::optional<FlightActionRequest> AutonomyRuntime::pending_aerial_yaw_stop(
    const mission::VehicleSnapshot& vehicle) {
    if (!aerial_yaw_stop_pending_ || !vehicle_system_id_.has_value()) {
        return std::nullopt;
    }
    if (vehicle.connected) {
        aerial_yaw_stop_pending_ = false;
    }
    return FlightActionRequest{
        .action = FlightAction::yaw_rate,
        .vehicle_system_id = vehicle.system_id.value_or(*vehicle_system_id_),
        .confirmation = 0,
        .yaw_rate_degrees_per_second = 0.0,
    };
}

void AutonomyRuntime::clear_aerial_yaw_guidance() {
    aerial_yaw_controller_.reset();
    last_aerial_observation_count_.reset();
    current_aerial_yaw_rate_degrees_per_second_.reset();
    aerial_horizontal_error_.reset();
    aerial_proportional_rate_degrees_per_second_.reset();
    aerial_feed_forward_rate_degrees_per_second_.reset();
    aerial_yaw_active_ = false;
}

bool AutonomyRuntime::prepare_active_runtime(
    const FlightStartupSnapshot& startup,
    const CompanionLinkFailsafeSnapshot& companion_link_failsafe) {
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
    detail_ = "Aerial observation active; holding after takeoff";
    return true;
}

bool AutonomyRuntime::validate_runtime_context(
    const mission::VehicleSnapshot& vehicle,
    const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
    const mission::TimePoint now) {
    if (!vehicle.connected || !vehicle.system_id.has_value()) {
        if (config_.mode == AutonomyRuntimeMode::aerial_observation &&
            (phase_ == AutonomyRuntimePhase::active ||
                phase_ == AutonomyRuntimePhase::suspended)) {
            return suspend_aerial_tracking_for_link_loss(now);
        }
        fail("Flight-controller heartbeat was lost during autonomy");
        return false;
    }

    aerial_link_loss_since_.reset();
    if (phase_ == AutonomyRuntimePhase::suspended &&
        !recover_suspended_aerial_tracking(companion_link_failsafe, now)) {
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

bool AutonomyRuntime::suspend_aerial_tracking_for_link_loss(
    const mission::TimePoint now) {
    aerial_link_recovered_since_.reset();
    if (!aerial_link_loss_since_.has_value()) {
        aerial_link_loss_since_ = now;
    }
    aerial_yaw_stop_pending_ = aerial_yaw_stop_pending_ || aerial_yaw_active_;
    clear_aerial_yaw_guidance();
    motion_safety_status_ = MotionSafetyStatus::flight_controller_disconnected;
    if (now - *aerial_link_loss_since_ >= config_.aerial_link_loss_grace) {
        fail("Flight-controller heartbeat remained unavailable during aerial "
             "tracking");
        return false;
    }

    phase_ = AutonomyRuntimePhase::suspended;
    detail_ = "CONTROLLER LINK INTERRUPTED | YAW PAUSED";
    return false;
}

bool AutonomyRuntime::recover_suspended_aerial_tracking(
    const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
    const mission::TimePoint now) {
    if (companion_link_failsafe.phase == CompanionLinkFailsafePhase::rejected) {
        fail("Companion-link failsafe is no longer valid: " +
             companion_link_failsafe.detail);
        return false;
    }
    if (!aerial_link_recovered_since_.has_value()) {
        aerial_link_recovered_since_ = now;
    }
    if (!companion_link_failsafe.accepted()) {
        detail_ = "CONTROLLER LINK RECOVERED | VERIFYING FAILSAFE";
        return false;
    }
    if (now - *aerial_link_recovered_since_ <
        config_.aerial_link_recovery_hold) {
        detail_ = "CONTROLLER LINK RECOVERED | VERIFYING STABILITY";
        return false;
    }

    phase_ = AutonomyRuntimePhase::active;
    aerial_link_recovered_since_.reset();
    clear_aerial_yaw_guidance();
    next_yaw_command_ = now;
    detail_ = "CONTROLLER LINK STABLE | TARGET SEARCHING";
    return true;
}

void AutonomyRuntime::on_action_sent(const FlightActionRequest& request,
    const bool sent,
    const mission::TimePoint) {
    if (request.action == FlightAction::return_to_launch) {
        if (!sent && phase_ == AutonomyRuntimePhase::returning_to_launch) {
            awaiting_rtl_ack_ = false;
            if (rtl_attempt_ >= kMaximumRtlAttempts) {
                fail("Failed to send RTL after 3 attempts");
            } else {
                detail_ = "Failed to send RTL; retrying";
            }
        }
        return;
    }

}

void AutonomyRuntime::on_command_ack(const FlightAction action,
    const FlightCommandAckOutcome outcome,
    const std::uint8_t raw_result,
    const std::uint8_t source_system,
    const mission::TimePoint) {
    if (action == FlightAction::return_to_launch &&
        phase_ == AutonomyRuntimePhase::returning_to_launch &&
        awaiting_rtl_ack_ && vehicle_system_id_.has_value() &&
        source_system == *vehicle_system_id_) {
        if (outcome == FlightCommandAckOutcome::rejected) {
            awaiting_rtl_ack_ = false;
            if (rtl_attempt_ >= kMaximumRtlAttempts) {
                failure_result_ = raw_result;
                fail("RTL was rejected with MAV_RESULT " +
                     std::to_string(raw_result));
            } else {
                detail_ = "RTL rejected; retrying";
            }
        } else {
            awaiting_rtl_ack_ = false;
            rtl_acknowledged_ = true;
            detail_ = outcome == FlightCommandAckOutcome::accepted
                          ? "RTL accepted; monitoring return"
                          : "RTL is in progress";
        }
    }
}

void AutonomyRuntime::restart() {
    if (!config_.enabled) {
        phase_ = AutonomyRuntimePhase::disabled;
        detail_ = "Autonomy runtime disabled";
    } else {
        phase_ = AutonomyRuntimePhase::waiting_for_startup;
        detail_ = "Preparing takeoff for aerial observation";
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
    const std::uint8_t vehicle_system_id,
    const mission::TimePoint now) {
    reset_runtime_state();
    phase_ = AutonomyRuntimePhase::returning_to_launch;
    detail_ = "RTL requested; waiting for ArduPilot";
    vehicle_system_id_ = vehicle_system_id;
    acknowledgement_deadline_ = now;
}

void AutonomyRuntime::reset_runtime_state() {
    clear_aerial_yaw_guidance();
    motion_safety_status_ = MotionSafetyStatus::no_intent;
    vehicle_system_id_.reset();
    next_yaw_command_ = {};
    acknowledgement_deadline_ = {};
    rtl_attempt_ = 0;
    awaiting_rtl_ack_ = false;
    rtl_acknowledged_ = false;
    aerial_yaw_stop_pending_ = false;
    aerial_link_loss_since_.reset();
    aerial_link_recovered_since_.reset();
    failure_result_.reset();
}

std::vector<FlightActionRequest> AutonomyRuntime::update_return_to_launch(
    const mission::VehicleSnapshot& vehicle,
    const mission::TimePoint now) {
    if (!vehicle.connected) {
        fail("Flight-controller heartbeat was lost during RTL");
        return {};
    }
    if (!vehicle_system_id_.has_value()) {
        fail("Cannot send RTL without a vehicle system ID");
        return {};
    }
    if (rtl_acknowledged_ && !vehicle.armed) {
        phase_ = AutonomyRuntimePhase::completed;
        detail_ = "RTL complete; vehicle disarmed";
        return {};
    }
    if (rtl_acknowledged_ ||
        (awaiting_rtl_ack_ && now < acknowledgement_deadline_)) {
        return {};
    }
    awaiting_rtl_ack_ = false;
    if (rtl_attempt_ >= kMaximumRtlAttempts) {
        fail("No COMMAND_ACK for RTL after 3 attempts");
        return {};
    }

    const auto confirmation = static_cast<std::uint8_t>(rtl_attempt_);
    ++rtl_attempt_;
    awaiting_rtl_ack_ = true;
    acknowledgement_deadline_ = now + kRtlAcknowledgementTimeout;
    detail_ = "RTL attempt " + std::to_string(rtl_attempt_) + "/3";
    return {{
        .action = FlightAction::return_to_launch,
        .vehicle_system_id = *vehicle_system_id_,
        .confirmation = confirmation,
    }};
}

AutonomyRuntimeSnapshot AutonomyRuntime::snapshot() const {
    return {
        .phase = phase_,
        .detail = detail_,
        .motion_safety_status = motion_safety_status_,
        .failure_result = failure_result_,
        .aerial_horizontal_error = aerial_horizontal_error_,
        .aerial_proportional_rate_degrees_per_second =
            aerial_proportional_rate_degrees_per_second_,
        .aerial_feed_forward_rate_degrees_per_second =
            aerial_feed_forward_rate_degrees_per_second_,
        .aerial_commanded_yaw_rate_degrees_per_second =
            current_aerial_yaw_rate_degrees_per_second_,
    };
}

void AutonomyRuntime::fail(std::string detail) {
    phase_ = AutonomyRuntimePhase::failed;
    detail_ = std::move(detail);
}

} // namespace onboard_autonomy::mission
