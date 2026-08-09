#pragma once

#include "onboard_autonomy/application/DecisionEngine.hpp"
#include "onboard_autonomy/application/CompanionLinkFailsafe.hpp"
#include "onboard_autonomy/application/FlightCommand.hpp"
#include "onboard_autonomy/application/FlightStartupController.hpp"
#include "onboard_autonomy/application/SafetySupervisor.hpp"
#include "onboard_autonomy/domain/VehicleState.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::application {

enum class AutonomyRuntimePhase {
    disabled,
    waiting_for_startup,
    active,
    landing,
    completed,
    failed,
};

struct AutonomyRuntimeConfig {
    bool enabled{false};
    std::chrono::milliseconds target_loss_land_after{
        std::chrono::seconds(5)
    };
    double terminal_descent_altitude_m{1.5};
    double terminal_alignment_radius_m{0.25};
    std::chrono::milliseconds terminal_alignment_duration{
        std::chrono::milliseconds(500)
    };
};

struct AutonomyRuntimeSnapshot {
    AutonomyRuntimePhase phase{AutonomyRuntimePhase::disabled};
    std::string detail{"Autonomy runtime disabled"};
    MotionSafetyStatus motion_safety_status{
        MotionSafetyStatus::no_intent
    };
    bool vision_landing_target_active{false};
    bool terminal_descent_active{false};
    std::size_t land_attempt{0};
    std::optional<std::uint8_t> failure_result;
};

class AutonomyRuntime {
public:
    explicit AutonomyRuntime(AutonomyRuntimeConfig config = {});

    [[nodiscard]] std::vector<FlightActionRequest> update(
        const domain::VehicleSnapshot& vehicle,
        const FlightStartupSnapshot& startup,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
        domain::TimePoint now,
        std::optional<domain::BodyFramePosition> landing_target =
            std::nullopt
    );

    void on_action_sent(
        const FlightActionRequest& request,
        bool sent,
        domain::TimePoint now
    );

    void on_command_ack(
        FlightAction action,
        FlightCommandAckOutcome outcome,
        std::uint8_t raw_result,
        std::uint8_t source_system,
        domain::TimePoint now
    );

    void restart();
    [[nodiscard]] AutonomyRuntimeSnapshot snapshot() const;

private:
    [[nodiscard]] std::optional<FlightActionRequest>
    update_land_command(domain::TimePoint now);
    void fail(std::string detail);

    static constexpr auto kLandingTargetInterval =
        std::chrono::milliseconds(100);
    static constexpr auto kLandingTargetWarmup =
        std::chrono::seconds(1);
    static constexpr auto kAcknowledgementTimeout =
        std::chrono::seconds(2);
    static constexpr auto kLandingTimeout =
        std::chrono::seconds(90);
    static constexpr std::size_t kMaximumLandAttempts = 3;
    static constexpr double kTargetStopAltitudeM = 0.20;

    AutonomyRuntimeConfig config_;
    AutonomyRuntimePhase phase_{AutonomyRuntimePhase::disabled};
    std::string detail_{"Autonomy runtime disabled"};
    DecisionEngine decision_engine_;
    SafetySupervisor safety_supervisor_;
    MotionSafetyStatus motion_safety_status_{
        MotionSafetyStatus::no_intent
    };
    std::optional<std::uint8_t> vehicle_system_id_;
    std::optional<domain::TimePoint> land_command_after_;
    std::optional<domain::TimePoint> target_missing_since_;
    std::optional<domain::TimePoint> terminal_alignment_since_;
    domain::TimePoint next_landing_target_{};
    domain::TimePoint acknowledgement_deadline_{};
    domain::TimePoint landing_deadline_{};
    std::size_t land_attempt_{0};
    bool awaiting_land_ack_{false};
    bool vision_landing_target_active_{false};
    bool terminal_alignment_confirmed_{false};
    bool terminal_descent_active_{false};
    std::optional<std::uint8_t> failure_result_;
};

}  // namespace onboard_autonomy::application
