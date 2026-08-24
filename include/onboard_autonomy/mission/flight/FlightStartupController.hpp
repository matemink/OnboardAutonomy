#pragma once

#include "onboard_autonomy/mission/safety/CompanionLinkFailsafe.hpp"
#include "onboard_autonomy/mission/flight/FlightCommand.hpp"
#include "onboard_autonomy/mission/flight/VehicleState.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::mission {

enum class FlightStartupPhase {
    disabled,
    idle,
    waiting_for_vehicle,
    waiting_for_readiness,
    setting_guided,
    arming,
    taking_off,
    completed,
    failed,
};

struct FlightStartupConfig {
    static constexpr double kDefaultTakeoffAltitudeM = 8.0;

    bool enabled{false};
    bool start_automatically{true};
    double takeoff_altitude_m{kDefaultTakeoffAltitudeM};
};

struct FlightStartupSnapshot {
    FlightStartupPhase phase{FlightStartupPhase::disabled};
    std::string detail{"Flight startup disabled"};
    double target_altitude_m{0.0};
    std::size_t attempt{0};
    std::optional<std::uint8_t> failure_result;
};

class FlightStartupController {
  public:
    explicit FlightStartupController(FlightStartupConfig config = {});

    [[nodiscard]] std::vector<FlightActionRequest> update(
        const mission::VehicleSnapshot& vehicle,
        bool telemetry_ready,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
        mission::TimePoint now);

    void on_action_sent(const FlightActionRequest& request,
        bool sent,
        mission::TimePoint now);

    void on_command_ack(FlightAction action,
        FlightCommandAckOutcome outcome,
        std::uint8_t raw_result,
        std::uint8_t source_system,
        mission::TimePoint now);

    void restart();
    void cancel(std::string detail);
    [[nodiscard]] FlightStartupSnapshot snapshot() const;

  private:
    enum class PhaseUpdate { advance, stop };

    [[nodiscard]] bool validate_active_context(
        const mission::VehicleSnapshot& vehicle,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe);
    [[nodiscard]] PhaseUpdate update_current_phase(
        const mission::VehicleSnapshot& vehicle,
        bool telemetry_ready,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
        mission::TimePoint now,
        std::vector<FlightActionRequest>& actions);
    [[nodiscard]] PhaseUpdate update_waiting_for_vehicle(
        const mission::VehicleSnapshot& vehicle,
        mission::TimePoint now);
    [[nodiscard]] PhaseUpdate update_waiting_for_readiness(
        const mission::VehicleSnapshot& vehicle,
        bool telemetry_ready,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
        mission::TimePoint now);
    [[nodiscard]] PhaseUpdate update_setting_guided(
        const mission::VehicleSnapshot& vehicle,
        mission::TimePoint now,
        std::vector<FlightActionRequest>& actions);
    [[nodiscard]] PhaseUpdate update_arming(
        const mission::VehicleSnapshot& vehicle,
        mission::TimePoint now,
        std::vector<FlightActionRequest>& actions);
    [[nodiscard]] PhaseUpdate update_taking_off(
        const mission::VehicleSnapshot& vehicle,
        mission::TimePoint now,
        std::vector<FlightActionRequest>& actions);
    void enter_phase(FlightStartupPhase phase, mission::TimePoint now);
    void fail(std::string detail);
    void update_readiness_detail(const mission::VehicleSnapshot& vehicle,
        bool telemetry_ready,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe);
    [[nodiscard]] std::optional<FlightActionRequest>
    update_command(FlightAction action, mission::TimePoint now);
    [[nodiscard]] std::optional<FlightAction> expected_action() const;

    FlightStartupConfig config_;
    FlightStartupPhase phase_{FlightStartupPhase::disabled};
    std::string detail_{"Flight startup disabled"};
    std::optional<std::uint8_t> vehicle_system_id_;
    std::size_t attempt_{0};
    bool awaiting_ack_{false};
    bool command_accepted_{false};
    mission::TimePoint acknowledgement_deadline_{};
    mission::TimePoint phase_deadline_{};
    std::optional<std::uint8_t> failure_result_;
};

} // namespace onboard_autonomy::mission
