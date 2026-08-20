#pragma once

#include "onboard_autonomy/application/CompanionLinkFailsafe.hpp"
#include "onboard_autonomy/application/FlightCommand.hpp"
#include "onboard_autonomy/domain/VehicleState.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::application {

enum class FlightStartupPhase {
    disabled,
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
        const domain::VehicleSnapshot& vehicle,
        bool telemetry_ready,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
        domain::TimePoint now);

    void on_action_sent(const FlightActionRequest& request,
        bool sent,
        domain::TimePoint now);

    void on_command_ack(FlightAction action,
        FlightCommandAckOutcome outcome,
        std::uint8_t raw_result,
        std::uint8_t source_system,
        domain::TimePoint now);

    void restart();
    [[nodiscard]] FlightStartupSnapshot snapshot() const;

  private:
    enum class PhaseUpdate { advance, stop };

    [[nodiscard]] bool validate_active_context(
        const domain::VehicleSnapshot& vehicle,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe);
    [[nodiscard]] PhaseUpdate update_current_phase(
        const domain::VehicleSnapshot& vehicle,
        bool telemetry_ready,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
        domain::TimePoint now,
        std::vector<FlightActionRequest>& actions);
    [[nodiscard]] PhaseUpdate update_waiting_for_vehicle(
        const domain::VehicleSnapshot& vehicle,
        domain::TimePoint now);
    [[nodiscard]] PhaseUpdate update_waiting_for_readiness(
        const domain::VehicleSnapshot& vehicle,
        bool telemetry_ready,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe,
        domain::TimePoint now);
    [[nodiscard]] PhaseUpdate update_setting_guided(
        const domain::VehicleSnapshot& vehicle,
        domain::TimePoint now,
        std::vector<FlightActionRequest>& actions);
    [[nodiscard]] PhaseUpdate update_arming(
        const domain::VehicleSnapshot& vehicle,
        domain::TimePoint now,
        std::vector<FlightActionRequest>& actions);
    [[nodiscard]] PhaseUpdate update_taking_off(
        const domain::VehicleSnapshot& vehicle,
        domain::TimePoint now,
        std::vector<FlightActionRequest>& actions);
    void enter_phase(FlightStartupPhase phase, domain::TimePoint now);
    void fail(std::string detail);
    void update_readiness_detail(const domain::VehicleSnapshot& vehicle,
        bool telemetry_ready,
        const CompanionLinkFailsafeSnapshot& companion_link_failsafe);
    [[nodiscard]] std::optional<FlightActionRequest>
    update_command(FlightAction action, domain::TimePoint now);
    [[nodiscard]] std::optional<FlightAction> expected_action() const;

    FlightStartupConfig config_;
    FlightStartupPhase phase_{FlightStartupPhase::disabled};
    std::string detail_{"Flight startup disabled"};
    std::optional<std::uint8_t> vehicle_system_id_;
    std::size_t attempt_{0};
    bool awaiting_ack_{false};
    bool command_accepted_{false};
    domain::TimePoint acknowledgement_deadline_{};
    domain::TimePoint phase_deadline_{};
    std::optional<std::uint8_t> failure_result_;
};

} // namespace onboard_autonomy::application
