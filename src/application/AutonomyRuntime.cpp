#include "onboard_autonomy/application/AutonomyRuntime.hpp"

#include "onboard_autonomy/application/WorldState.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace onboard_autonomy::application {

AutonomyRuntime::AutonomyRuntime(AutonomyRuntimeConfig config)
    : config_(std::move(config)) {
    if (config_.target_loss_land_after <=
        std::chrono::milliseconds::zero()) {
        throw std::invalid_argument(
            "target-loss LAND timeout must be positive"
        );
    }
    if (!std::isfinite(config_.terminal_descent_altitude_m) ||
        config_.terminal_descent_altitude_m <= 0.0 ||
        !std::isfinite(config_.terminal_alignment_radius_m) ||
        config_.terminal_alignment_radius_m <= 0.0 ||
        config_.terminal_alignment_duration <=
            std::chrono::milliseconds::zero()) {
        throw std::invalid_argument(
            "terminal-descent thresholds must be positive"
        );
    }
    if (config_.enabled) {
        restart();
    }
}

std::vector<FlightActionRequest> AutonomyRuntime::update(
    const domain::VehicleSnapshot& vehicle,
    const FlightStartupSnapshot& startup,
    const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
    const domain::TimePoint now,
    std::optional<domain::BodyFramePosition> landing_target
) {
    std::vector<FlightActionRequest> actions;
    if (phase_ == AutonomyRuntimePhase::disabled ||
        phase_ == AutonomyRuntimePhase::completed ||
        phase_ == AutonomyRuntimePhase::failed) {
        return actions;
    }

    if (phase_ == AutonomyRuntimePhase::waiting_for_startup) {
        if (startup.phase == FlightStartupPhase::failed) {
            fail("Flight startup failed: " + startup.detail);
            return actions;
        }
        if (startup.phase != FlightStartupPhase::completed) {
            detail_ = "Waiting for verified flight startup";
            return actions;
        }
        if (!companion_link_failsafe.accepted()) {
            fail(
                "Companion-link failsafe is not valid: " +
                companion_link_failsafe.detail
            );
            return actions;
        }
        phase_ = AutonomyRuntimePhase::active;
        detail_ = "Autonomy active; waiting for vision target";
        target_missing_since_ = now;
        next_landing_target_ = now;
    }

    if (!vehicle.connected || !vehicle.system_id.has_value()) {
        fail("Flight-controller heartbeat was lost during autonomy");
        return actions;
    }
    if (!companion_link_failsafe.accepted()) {
        fail(
            "Companion-link failsafe is no longer valid: " +
            companion_link_failsafe.detail
        );
        return actions;
    }
    vehicle_system_id_ = vehicle.system_id;

    if (phase_ == AutonomyRuntimePhase::landing && !vehicle.armed) {
        phase_ = AutonomyRuntimePhase::completed;
        detail_ = "Landing complete; vehicle disarmed";
        vision_landing_target_active_ = false;
        return actions;
    }
    if (phase_ == AutonomyRuntimePhase::active && !vehicle.armed) {
        fail("Vehicle disarmed while autonomy was active");
        return actions;
    }
    if (phase_ == AutonomyRuntimePhase::landing &&
        now >= landing_deadline_) {
        fail("Vehicle did not complete landing");
        return actions;
    }
    if (phase_ == AutonomyRuntimePhase::landing &&
        terminal_descent_active_) {
        vision_landing_target_active_ = false;
        detail_ =
            "Terminal descent; vision corrections are latched off";
        return actions;
    }

    if (phase_ == AutonomyRuntimePhase::landing &&
        vehicle.relative_altitude_m.has_value() &&
        *vehicle.relative_altitude_m <=
            config_.terminal_descent_altitude_m &&
        landing_target.has_value()) {
        const double lateral_error_m = std::hypot(
            landing_target->forward_m,
            landing_target->right_m
        );
        if (lateral_error_m <=
            config_.terminal_alignment_radius_m) {
            if (!terminal_alignment_since_.has_value()) {
                terminal_alignment_since_ = now;
            }
            terminal_alignment_confirmed_ =
                now - *terminal_alignment_since_ >=
                config_.terminal_alignment_duration;
        } else {
            terminal_alignment_since_.reset();
            terminal_alignment_confirmed_ = false;
        }
    } else if (landing_target.has_value()) {
        terminal_alignment_since_.reset();
        terminal_alignment_confirmed_ = false;
    }
    if (phase_ == AutonomyRuntimePhase::landing &&
        vehicle.relative_altitude_m.has_value() &&
        *vehicle.relative_altitude_m <= kTargetStopAltitudeM) {
        vision_landing_target_active_ = false;
        detail_ =
            "Touchdown detected; waiting for ArduPilot auto-disarm";
        return actions;
    }

    auto world = make_world_state(
        vehicle,
        std::move(landing_target),
        now
    );
    const auto desired = decision_engine_.decide(world);
    const auto supervised = safety_supervisor_.supervise(
        world,
        desired,
        now
    );
    motion_safety_status_ = supervised.status;

    if (!supervised.approved.has_value()) {
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
                detail_ =
                    "Terminal descent; vision corrections are latched off";
            } else {
                terminal_alignment_since_.reset();
                terminal_alignment_confirmed_ = false;
                detail_ =
                    "Landing without a fresh vision target";
            }
            return actions;
        }

        if (now - *target_missing_since_ >=
            config_.target_loss_land_after) {
            detail_ = "Vision unavailable; requesting fallback LAND";
            if (auto land = update_land_command(now)) {
                actions.push_back(*land);
            }
        } else {
            detail_ = "Waiting for a fresh confirmed vision target";
        }
        return actions;
    }

    target_missing_since_.reset();
    const auto& intent = *supervised.approved;
    if (!land_command_after_.has_value() &&
        phase_ == AutonomyRuntimePhase::active) {
        land_command_after_ = now + kLandingTargetWarmup;
        next_landing_target_ = now;
    }

    if (now >= next_landing_target_) {
        const auto elapsed = std::chrono::duration_cast<
            std::chrono::microseconds
        >(now.time_since_epoch());
        actions.push_back(
            FlightActionRequest{
                .action = FlightAction::landing_target,
                .vehicle_system_id = intent.vehicle_system_id,
                .x_m = intent.landing_target.forward_m,
                .y_m = intent.landing_target.right_m,
                .z_m = intent.landing_target.down_m,
                .time_usec =
                    static_cast<std::uint64_t>(elapsed.count()),
            }
        );
        next_landing_target_ = now + kLandingTargetInterval;
    }

    if (phase_ == AutonomyRuntimePhase::active &&
        land_command_after_.has_value() &&
        now >= *land_command_after_) {
        if (auto land = update_land_command(now)) {
            actions.push_back(*land);
        }
    }

    std::ostringstream target;
    target << "Vision target F/R/D "
           << std::fixed << std::setprecision(1)
           << intent.landing_target.forward_m << "/"
           << intent.landing_target.right_m << "/"
           << intent.landing_target.down_m << " m";
    detail_ = target.str();
    return actions;
}

void AutonomyRuntime::on_action_sent(
    const FlightActionRequest& request,
    const bool sent,
    const domain::TimePoint
) {
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

void AutonomyRuntime::on_command_ack(
    const FlightAction action,
    const FlightCommandAckOutcome outcome,
    const std::uint8_t raw_result,
    const std::uint8_t source_system,
    const domain::TimePoint now
) {
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
            acknowledgement_deadline_ =
                now + kAcknowledgementTimeout;
            detail_ = "LAND is in progress";
            break;
        case FlightCommandAckOutcome::rejected:
            failure_result_ = raw_result;
            fail(
                "LAND was rejected with MAV_RESULT " +
                std::to_string(raw_result)
            );
            break;
    }
}

void AutonomyRuntime::restart() {
    if (!config_.enabled) {
        phase_ = AutonomyRuntimePhase::disabled;
        detail_ = "Autonomy runtime disabled";
    } else {
        phase_ = AutonomyRuntimePhase::waiting_for_startup;
        detail_ = "Waiting for verified flight startup";
    }
    motion_safety_status_ = MotionSafetyStatus::no_intent;
    vehicle_system_id_.reset();
    land_command_after_.reset();
    target_missing_since_.reset();
    terminal_alignment_since_.reset();
    next_landing_target_ = {};
    acknowledgement_deadline_ = {};
    landing_deadline_ = {};
    land_attempt_ = 0;
    awaiting_land_ack_ = false;
    vision_landing_target_active_ = false;
    terminal_alignment_confirmed_ = false;
    terminal_descent_active_ = false;
    failure_result_.reset();
}

AutonomyRuntimeSnapshot AutonomyRuntime::snapshot() const {
    return {
        .phase = phase_,
        .detail = detail_,
        .motion_safety_status = motion_safety_status_,
        .vision_landing_target_active =
            vision_landing_target_active_,
        .terminal_descent_active = terminal_descent_active_,
        .land_attempt = land_attempt_,
        .failure_result = failure_result_,
    };
}

std::optional<FlightActionRequest>
AutonomyRuntime::update_land_command(const domain::TimePoint now) {
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

    const auto confirmation =
        static_cast<std::uint8_t>(land_attempt_);
    ++land_attempt_;
    awaiting_land_ack_ = true;
    acknowledgement_deadline_ = now + kAcknowledgementTimeout;
    return FlightActionRequest{
        .action = FlightAction::land,
        .vehicle_system_id = *vehicle_system_id_,
        .confirmation = confirmation,
    };
}

void AutonomyRuntime::fail(std::string detail) {
    phase_ = AutonomyRuntimePhase::failed;
    detail_ = std::move(detail);
    awaiting_land_ack_ = false;
    vision_landing_target_active_ = false;
}

}  // namespace onboard_autonomy::application
