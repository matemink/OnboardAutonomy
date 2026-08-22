#include "TestCases.hpp"

#include "onboard_autonomy/mission/flight/FlightStartupController.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using onboard_autonomy::mission::CompanionLinkFailsafePhase;
using onboard_autonomy::mission::CompanionLinkFailsafeSnapshot;
using onboard_autonomy::mission::FlightAction;
using onboard_autonomy::mission::FlightActionRequest;
using onboard_autonomy::mission::FlightCommandAckOutcome;
using onboard_autonomy::mission::FlightStartupController;
using onboard_autonomy::mission::FlightStartupPhase;
using onboard_autonomy::mission::TimePoint;
using onboard_autonomy::mission::VehicleSnapshot;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

VehicleSnapshot ready_vehicle() {
    VehicleSnapshot vehicle;
    vehicle.connected = true;
    vehicle.gps_ready = true;
    vehicle.navigation_ready = true;
    vehicle.battery_ready = true;
    vehicle.system_health_known = true;
    vehicle.system_health_ok = true;
    vehicle.armable = true;
    vehicle.system_id = 1;
    vehicle.vehicle_type = 2;
    vehicle.autopilot_type = 3;
    vehicle.flight_mode = 0;
    vehicle.relative_altitude_m = 0.0;
    return vehicle;
}

CompanionLinkFailsafeSnapshot accepted_failsafe() {
    CompanionLinkFailsafeSnapshot failsafe;
    failsafe.phase = CompanionLinkFailsafePhase::accepted;
    failsafe.detail = "ArduPilot LAND policy accepted";
    return failsafe;
}

FlightActionRequest only_action(const std::vector<FlightActionRequest>& actions,
    const FlightAction expected,
    const std::string& message) {
    require(actions.size() == 1 && actions.front().action == expected, message);
    return actions.front();
}

void accept(FlightStartupController& controller,
    const FlightActionRequest& request,
    const TimePoint now) {
    controller.on_action_sent(request, true, now);
    controller.on_command_ack(request.action,
        FlightCommandAckOutcome::accepted,
        0,
        1,
        now);
}

void startup_ends_after_verified_takeoff() {
    FlightStartupController controller{
        {.enabled = true, .takeoff_altitude_m = 8.0}};
    auto vehicle = ready_vehicle();
    const TimePoint start{};

    auto action = only_action(
        controller.update(vehicle, true, accepted_failsafe(), start),
        FlightAction::set_guided_mode,
        "startup must begin with GUIDED");
    accept(controller, action, start);
    vehicle.flight_mode = 4;

    action = only_action(controller.update(vehicle,
                             true,
                             accepted_failsafe(),
                             start + std::chrono::milliseconds(100)),
        FlightAction::arm,
        "confirmed GUIDED must lead to ARM");
    accept(controller, action, start + std::chrono::milliseconds(100));
    vehicle.armed = true;

    action = only_action(controller.update(vehicle,
                             true,
                             accepted_failsafe(),
                             start + std::chrono::milliseconds(200)),
        FlightAction::takeoff,
        "confirmed ARMED must lead to TAKEOFF");
    require(action.altitude_m == 8.0,
        "startup must carry configured safe altitude");
    accept(controller, action, start + std::chrono::milliseconds(200));

    vehicle.relative_altitude_m = 7.7;
    require(controller
                .update(vehicle,
                    true,
                    accepted_failsafe(),
                    start + std::chrono::seconds(3))
                .empty(),
        "altitude confirmation must not emit another command");
    require(controller.snapshot().phase == FlightStartupPhase::completed,
        "startup responsibility must end after verified takeoff");
}

void startup_fails_after_three_missing_acknowledgements() {
    FlightStartupController controller{
        {.enabled = true, .takeoff_altitude_m = 5.0}};
    const auto vehicle = ready_vehicle();
    const TimePoint start{};

    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto action =
            only_action(controller.update(vehicle,
                            true,
                            accepted_failsafe(),
                            start + std::chrono::seconds(attempt * 3)),
                FlightAction::set_guided_mode,
                "missing GUIDED ACK must retry");
        require(action.confirmation == static_cast<std::uint8_t>(attempt),
            "retry confirmation must increase");
    }

    static_cast<void>(controller.update(vehicle,
        true,
        accepted_failsafe(),
        start + std::chrono::seconds(9)));
    require(controller.snapshot().phase == FlightStartupPhase::failed,
        "startup must fail after ACK retry exhaustion");
}

void startup_stops_on_heartbeat_loss() {
    FlightStartupController controller{
        {.enabled = true, .takeoff_altitude_m = 5.0}};
    auto vehicle = ready_vehicle();
    const TimePoint start{};
    static_cast<void>(
        controller.update(vehicle, false, accepted_failsafe(), start));

    vehicle.connected = false;
    static_cast<void>(controller.update(vehicle,
        false,
        accepted_failsafe(),
        start + std::chrono::seconds(1)));
    require(controller.snapshot().phase == FlightStartupPhase::failed,
        "heartbeat loss must stop startup commands");
}

void startup_waits_for_accepted_link_failsafe() {
    FlightStartupController controller{
        {.enabled = true, .takeoff_altitude_m = 5.0}};
    CompanionLinkFailsafeSnapshot rejected;
    rejected.phase = CompanionLinkFailsafePhase::rejected;
    rejected.detail = "FS_GCS_ENABLE must be 5 (Always LAND)";

    require(
        controller.update(ready_vehicle(), true, rejected, TimePoint{}).empty(),
        "rejected ArduPilot failsafe must block startup commands");
    require(controller.snapshot().detail.find("Autonomy blocked") !=
                std::string::npos,
        "startup must expose the failsafe rejection reason");
}

void startup_uses_source_neutral_navigation_readiness() {
    FlightStartupController controller{
        {.enabled = true, .takeoff_altitude_m = 5.0}};
    auto vehicle = ready_vehicle();
    vehicle.gps_ready = false;

    const auto action = only_action(
        controller.update(vehicle, true, accepted_failsafe(), TimePoint{}),
        FlightAction::set_guided_mode,
        "ready external navigation must not require GPS");
    require(action.action == FlightAction::set_guided_mode,
        "source-neutral navigation must allow startup to advance");
}

void startup_reports_missing_navigation_estimate() {
    FlightStartupController controller{
        {.enabled = true, .takeoff_altitude_m = 5.0}};
    auto vehicle = ready_vehicle();
    vehicle.gps_ready = false;
    vehicle.navigation_ready = false;
    vehicle.armable = false;

    require(controller
                .update(vehicle, true, accepted_failsafe(), TimePoint{})
                .empty(),
        "missing navigation must block startup commands");
    require(controller.snapshot().detail ==
                "Waiting for a navigation estimate",
        "startup must describe the source-neutral navigation requirement");
}

void restart_clears_terminal_startup_state() {
    FlightStartupController controller{
        {.enabled = true, .takeoff_altitude_m = 5.0}};
    auto vehicle = ready_vehicle();
    static_cast<void>(
        controller.update(vehicle, false, accepted_failsafe(), TimePoint{}));
    vehicle.connected = false;
    static_cast<void>(controller.update(vehicle,
        false,
        accepted_failsafe(),
        TimePoint{} + std::chrono::seconds(1)));
    require(controller.snapshot().phase == FlightStartupPhase::failed,
        "test setup must reach a terminal startup state");

    controller.restart();

    const auto snapshot = controller.snapshot();
    require(snapshot.phase == FlightStartupPhase::waiting_for_vehicle &&
                snapshot.attempt == 0 && !snapshot.failure_result.has_value(),
        "restart must reset startup state for another flight");
}

} // namespace

void run_flight_startup_controller_tests() {
    startup_ends_after_verified_takeoff();
    startup_fails_after_three_missing_acknowledgements();
    startup_stops_on_heartbeat_loss();
    startup_waits_for_accepted_link_failsafe();
    startup_uses_source_neutral_navigation_readiness();
    startup_reports_missing_navigation_estimate();
    restart_clears_terminal_startup_state();
}
