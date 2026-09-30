#pragma once

#include "onboard_autonomy/mission/safety/CompanionLinkFailsafe.hpp"
#include "onboard_autonomy/mission/autonomy/AerialYawController.hpp"
#include "onboard_autonomy/mission/cv/tracking/AerialTargetTracker.hpp"
#include "onboard_autonomy/mission/flight/FlightCommand.hpp"
#include "onboard_autonomy/mission/flight/FlightStartupController.hpp"
#include "onboard_autonomy/mission/safety/MotionSafetyStatus.hpp"
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
    suspended,
    returning_to_launch,
    completed,
    failed,
};

enum class AutonomyRuntimeMode {
    aerial_observation,
};

struct AutonomyRuntimeConfig {
    static constexpr auto kDefaultYawCommandInterval =
        std::chrono::milliseconds{100};
    static constexpr double kDefaultYawDeadbandRatio = 0.08;
    static constexpr double kDefaultMaximumYawStepDegrees = 10.0;
    static constexpr double kDefaultYawSpeedDegreesPerSecond = 90.0;
    static constexpr double kDefaultForwardCameraHorizontalFovRadians = 2.0;
    static constexpr auto kDefaultAerialLinkLossGrace = std::chrono::seconds{1};
    static constexpr auto kDefaultAerialLinkRecoveryHold =
        std::chrono::milliseconds{500};

    bool enabled{false};
    bool start_automatically{true};
    AutonomyRuntimeMode mode{AutonomyRuntimeMode::aerial_observation};
    std::chrono::milliseconds yaw_command_interval{kDefaultYawCommandInterval};
    double yaw_deadband_ratio{kDefaultYawDeadbandRatio};
    double maximum_yaw_step_degrees{kDefaultMaximumYawStepDegrees};
    double yaw_speed_degrees_per_second{kDefaultYawSpeedDegreesPerSecond};
    double forward_camera_horizontal_fov_radians{
        kDefaultForwardCameraHorizontalFovRadians};
    std::chrono::milliseconds aerial_link_loss_grace{
        kDefaultAerialLinkLossGrace};
    std::chrono::milliseconds aerial_link_recovery_hold{
        kDefaultAerialLinkRecoveryHold};
};

struct AutonomyRuntimeSnapshot {
    AutonomyRuntimePhase phase{AutonomyRuntimePhase::disabled};
    std::string detail{"Autonomy runtime disabled"};
    MotionSafetyStatus motion_safety_status{MotionSafetyStatus::no_intent};
    std::optional<std::uint8_t> failure_result;
    std::optional<double> aerial_horizontal_error;
    std::optional<double> aerial_proportional_rate_degrees_per_second;
    std::optional<double> aerial_feed_forward_rate_degrees_per_second;
    std::optional<double> aerial_commanded_yaw_rate_degrees_per_second;
};

class AutonomyRuntime {
  public:
    explicit AutonomyRuntime(const AutonomyRuntimeConfig& config = {});

    [[nodiscard]] std::vector<FlightActionRequest> update(
        const mission::VehicleSnapshot& vehicle,
        const FlightStartupSnapshot& startup,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
        mission::TimePoint now,
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
    void begin_return_to_launch(std::uint8_t vehicle_system_id,
        mission::TimePoint now);
    [[nodiscard]] AutonomyRuntimeSnapshot snapshot() const;

  private:
    [[nodiscard]] bool prepare_active_runtime(
        const FlightStartupSnapshot& startup,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe);
    [[nodiscard]] bool validate_runtime_context(
        const mission::VehicleSnapshot& vehicle,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
        mission::TimePoint now);
    [[nodiscard]] bool suspend_aerial_tracking_for_link_loss(
        mission::TimePoint now);
    [[nodiscard]] bool recover_suspended_aerial_tracking(
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
        mission::TimePoint now);
    [[nodiscard]] std::vector<FlightActionRequest> update_aerial_observation(
        const mission::VehicleSnapshot& vehicle,
        mission::TimePoint now,
        const std::optional<AerialTargetTrackSnapshot>& aerial_target);
    [[nodiscard]] std::optional<FlightActionRequest> pending_aerial_yaw_stop(
        const mission::VehicleSnapshot& vehicle);
    [[nodiscard]] std::vector<FlightActionRequest> stop_aerial_yaw();
    void clear_aerial_yaw_guidance();
    [[nodiscard]] std::vector<FlightActionRequest> update_return_to_launch(
        const mission::VehicleSnapshot& vehicle,
        mission::TimePoint now);
    void reset_runtime_state();
    void fail(std::string detail);

    static constexpr auto kRtlAcknowledgementTimeout = std::chrono::seconds(2);
    static constexpr std::size_t kMaximumRtlAttempts = 3;

    AutonomyRuntimeConfig config_;
    AerialYawController aerial_yaw_controller_;
    AutonomyRuntimePhase phase_{AutonomyRuntimePhase::disabled};
    std::string detail_{"Autonomy runtime disabled"};
    MotionSafetyStatus motion_safety_status_{MotionSafetyStatus::no_intent};
    std::optional<std::uint8_t> vehicle_system_id_;
    mission::TimePoint next_yaw_command_{};
    mission::TimePoint acknowledgement_deadline_{};
    std::size_t rtl_attempt_{0};
    bool awaiting_rtl_ack_{false};
    bool rtl_acknowledged_{false};
    bool aerial_yaw_active_{false};
    bool aerial_yaw_stop_pending_{false};
    std::optional<std::uint64_t> last_aerial_observation_count_;
    std::optional<double> current_aerial_yaw_rate_degrees_per_second_;
    std::optional<double> aerial_horizontal_error_;
    std::optional<double> aerial_proportional_rate_degrees_per_second_;
    std::optional<double> aerial_feed_forward_rate_degrees_per_second_;
    std::optional<mission::TimePoint> aerial_link_loss_since_;
    std::optional<mission::TimePoint> aerial_link_recovered_since_;
    std::optional<std::uint8_t> failure_result_;
};

} // namespace onboard_autonomy::mission
