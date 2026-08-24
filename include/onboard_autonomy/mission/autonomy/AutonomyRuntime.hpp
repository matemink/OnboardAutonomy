#pragma once

#include "onboard_autonomy/mission/safety/CompanionLinkFailsafe.hpp"
#include "onboard_autonomy/mission/autonomy/DecisionEngine.hpp"
#include "onboard_autonomy/mission/cv/tracking/AerialTargetTracker.hpp"
#include "onboard_autonomy/mission/flight/FlightCommand.hpp"
#include "onboard_autonomy/mission/flight/FlightStartupController.hpp"
#include "onboard_autonomy/mission/safety/SafetySupervisor.hpp"
#include "onboard_autonomy/mission/flight/VehicleState.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::mission {

enum class AutonomyRuntimePhase {
    disabled,
    idle,
    waiting_for_startup,
    active,
    landing,
    returning_to_launch,
    completed,
    failed,
};

enum class AutonomyRuntimeMode {
    precision_landing,
    aerial_observation,
};

struct AutonomyRuntimeConfig {
    static constexpr auto kDefaultTargetLossLandAfter = std::chrono::seconds{5};
    static constexpr double kDefaultTerminalDescentAltitudeM = 1.5;
    static constexpr double kDefaultTerminalAlignmentRadiusM = 0.25;
    static constexpr auto kDefaultTerminalAlignmentDuration =
        std::chrono::milliseconds{500};
    static constexpr auto kDefaultYawCommandInterval =
        std::chrono::milliseconds{500};
    static constexpr double kDefaultYawDeadbandRatio = 0.08;
    static constexpr double kDefaultMaximumYawStepDegrees = 10.0;
    static constexpr double kDefaultYawSpeedDegreesPerSecond = 15.0;
    static constexpr double kDefaultForwardCameraHorizontalFovRadians = 2.0;

    bool enabled{false};
    bool start_automatically{true};
    AutonomyRuntimeMode mode{AutonomyRuntimeMode::precision_landing};
    std::chrono::milliseconds target_loss_land_after{
        kDefaultTargetLossLandAfter};
    double terminal_descent_altitude_m{kDefaultTerminalDescentAltitudeM};
    double terminal_alignment_radius_m{kDefaultTerminalAlignmentRadiusM};
    std::chrono::milliseconds terminal_alignment_duration{
        kDefaultTerminalAlignmentDuration};
    std::chrono::milliseconds yaw_command_interval{kDefaultYawCommandInterval};
    double yaw_deadband_ratio{kDefaultYawDeadbandRatio};
    double maximum_yaw_step_degrees{kDefaultMaximumYawStepDegrees};
    double yaw_speed_degrees_per_second{kDefaultYawSpeedDegreesPerSecond};
    double forward_camera_horizontal_fov_radians{
        kDefaultForwardCameraHorizontalFovRadians};
};

struct AutonomyRuntimeSnapshot {
    AutonomyRuntimePhase phase{AutonomyRuntimePhase::disabled};
    std::string detail{"Autonomy runtime disabled"};
    MotionSafetyStatus motion_safety_status{MotionSafetyStatus::no_intent};
    bool vision_landing_target_active{false};
    bool terminal_descent_active{false};
    std::size_t land_attempt{0};
    std::optional<std::uint8_t> failure_result;
};

class AutonomyRuntime {
  public:
    explicit AutonomyRuntime(const AutonomyRuntimeConfig& config = {});

    [[nodiscard]] std::vector<FlightActionRequest> update(
        const mission::VehicleSnapshot& vehicle,
        const FlightStartupSnapshot& startup,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
        mission::TimePoint now,
        std::optional<mission::BodyFramePosition> landing_target = std::nullopt,
        std::optional<AerialTargetTrackSnapshot> aerial_target = std::nullopt);

    void on_action_sent(const FlightActionRequest& request,
        bool sent,
        mission::TimePoint now);

    void on_command_ack(FlightAction action,
        FlightCommandAckOutcome outcome,
        std::uint8_t raw_result,
        std::uint8_t source_system,
        mission::TimePoint now);

    void restart();
    void restart(AutonomyRuntimeMode mode);
    void cancel(std::string detail);
    void begin_return_to_launch(std::uint8_t vehicle_system_id);
    [[nodiscard]] AutonomyRuntimeSnapshot snapshot() const;

  private:
    [[nodiscard]] bool prepare_active_runtime(
        const FlightStartupSnapshot& startup,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
        mission::TimePoint now);
    [[nodiscard]] bool validate_runtime_context(
        const mission::VehicleSnapshot& vehicle,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe);
    [[nodiscard]] bool continue_landing_update(
        const mission::VehicleSnapshot& vehicle,
        mission::TimePoint now,
        const std::optional<mission::BodyFramePosition>& landing_target);
    [[nodiscard]] std::vector<FlightActionRequest> update_aerial_observation(
        const mission::VehicleSnapshot& vehicle,
        mission::TimePoint now,
        const std::optional<AerialTargetTrackSnapshot>& aerial_target);
    void update_terminal_alignment(const mission::VehicleSnapshot& vehicle,
        mission::TimePoint now,
        const std::optional<mission::BodyFramePosition>& landing_target);
    [[nodiscard]] std::vector<FlightActionRequest> handle_missing_target(
        const mission::VehicleSnapshot& vehicle,
        mission::TimePoint now);
    [[nodiscard]] std::vector<FlightActionRequest>
    handle_approved_motion(const DesiredMotion& intent, mission::TimePoint now);
    [[nodiscard]] std::optional<FlightActionRequest> update_land_command(
        mission::TimePoint now);
    [[nodiscard]] std::vector<FlightActionRequest> update_return_to_launch(
        const mission::VehicleSnapshot& vehicle);
    void reset_runtime_state();
    void fail(std::string detail);

    static constexpr auto kLandingTargetInterval =
        std::chrono::milliseconds(100);
    static constexpr auto kLandingTargetWarmup = std::chrono::seconds(1);
    static constexpr auto kAcknowledgementTimeout = std::chrono::seconds(2);
    static constexpr auto kLandingTimeout = std::chrono::seconds(90);
    static constexpr std::size_t kMaximumLandAttempts = 3;
    static constexpr double kTargetStopAltitudeM = 0.20;

    AutonomyRuntimeConfig config_;
    AutonomyRuntimePhase phase_{AutonomyRuntimePhase::disabled};
    std::string detail_{"Autonomy runtime disabled"};
    DecisionEngine decision_engine_;
    SafetySupervisor safety_supervisor_;
    MotionSafetyStatus motion_safety_status_{MotionSafetyStatus::no_intent};
    std::optional<std::uint8_t> vehicle_system_id_;
    std::optional<mission::TimePoint> land_command_after_;
    std::optional<mission::TimePoint> target_missing_since_;
    std::optional<mission::TimePoint> terminal_alignment_since_;
    mission::TimePoint next_landing_target_{};
    mission::TimePoint next_yaw_command_{};
    mission::TimePoint acknowledgement_deadline_{};
    mission::TimePoint landing_deadline_{};
    std::size_t land_attempt_{0};
    bool awaiting_land_ack_{false};
    bool vision_landing_target_active_{false};
    bool terminal_alignment_confirmed_{false};
    bool terminal_descent_active_{false};
    std::optional<std::uint8_t> failure_result_;
};

} // namespace onboard_autonomy::mission
