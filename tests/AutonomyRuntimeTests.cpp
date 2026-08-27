#include "TestCases.hpp"

#include "onboard_autonomy/mission/autonomy/AutonomyRuntime.hpp"

#include <algorithm>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using onboard_autonomy::mission::AerialTargetTrackPhase;
using onboard_autonomy::mission::AerialTargetTrackSnapshot;
using onboard_autonomy::mission::AutonomyRuntime;
using onboard_autonomy::mission::AutonomyRuntimeMode;
using onboard_autonomy::mission::AutonomyRuntimePhase;
using onboard_autonomy::mission::BodyFramePosition;
using onboard_autonomy::mission::CompanionLinkFailsafePhase;
using onboard_autonomy::mission::CompanionLinkFailsafeSnapshot;
using onboard_autonomy::mission::FlightAction;
using onboard_autonomy::mission::FlightActionRequest;
using onboard_autonomy::mission::FlightCommandAckOutcome;
using onboard_autonomy::mission::FlightStartupPhase;
using onboard_autonomy::mission::FlightStartupSnapshot;
using onboard_autonomy::mission::TimePoint;
using onboard_autonomy::mission::VehicleSnapshot;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

VehicleSnapshot flying_vehicle() {
    VehicleSnapshot vehicle;
    vehicle.connected = true;
    vehicle.armed = true;
    vehicle.yaw_rad = 0.0;
    vehicle.yaw_rate_rad_per_second = 0.0;
    vehicle.system_id = 1;
    vehicle.relative_altitude_m = 8.0;
    return vehicle;
}

FlightStartupSnapshot completed_startup() {
    FlightStartupSnapshot startup;
    startup.phase = FlightStartupPhase::completed;
    startup.detail = "Takeoff complete";
    return startup;
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

void enter_landing(AutonomyRuntime& runtime,
    const VehicleSnapshot& vehicle,
    const FlightStartupSnapshot& startup,
    const CompanionLinkFailsafeSnapshot& failsafe,
    const TimePoint start,
    const BodyFramePosition& target) {
    static_cast<void>(
        runtime.update(vehicle, startup, failsafe, start, target));
    const auto warm_actions = runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::seconds(1),
        target);
    const auto land = std::find_if(warm_actions.begin(),
        warm_actions.end(),
        [](const FlightActionRequest& request) {
            return request.action == FlightAction::land;
        });
    require(land != warm_actions.end(), "test setup must request LAND");
    runtime.on_action_sent(*land, true, start + std::chrono::seconds(1));
    runtime.on_command_ack(FlightAction::land,
        FlightCommandAckOutcome::accepted,
        0,
        1,
        start + std::chrono::seconds(1));
}

void runtime_waits_for_startup() {
    AutonomyRuntime runtime{{.enabled = true}};
    auto startup = completed_startup();
    startup.phase = FlightStartupPhase::taking_off;

    require(
        runtime
            .update(flying_vehicle(), startup, accepted_failsafe(), TimePoint{})
            .empty(),
        "runtime must not bypass flight startup");
    require(runtime.snapshot().phase ==
                AutonomyRuntimePhase::waiting_for_startup,
        "runtime must expose startup dependency");
}

void aerial_observation_holds_after_takeoff_without_landing() {
    AutonomyRuntime runtime{{
        .enabled = true,
        .mode = AutonomyRuntimeMode::aerial_observation,
    }};
    const auto vehicle = flying_vehicle();
    const auto startup = completed_startup();
    const auto failsafe = accepted_failsafe();
    const TimePoint start{};

    require(runtime.snapshot().detail ==
                "Preparing takeoff for aerial observation",
        "observation mode must be visible before flight startup completes");
    require(runtime.update(vehicle, startup, failsafe, start).empty(),
        "aerial observation must hold without emitting a motion command");
    require(runtime
                .update(vehicle,
                    startup,
                    failsafe,
                    start + std::chrono::seconds(10))
                .empty(),
        "missing landing vision must not trigger LAND in observation mode");
    const auto snapshot = runtime.snapshot();
    require(snapshot.phase == AutonomyRuntimePhase::active &&
                snapshot.detail == "TARGET SEARCHING | GUIDED HOLD" &&
                snapshot.land_attempt == 0,
        "observation mode must expose a stable active hold");
}

void aerial_observation_yaws_only_for_a_stable_target_lock() {
    AutonomyRuntime runtime{{
        .enabled = true,
        .mode = AutonomyRuntimeMode::aerial_observation,
    }};
    const auto vehicle = flying_vehicle();
    const auto startup = completed_startup();
    const auto failsafe = accepted_failsafe();
    const TimePoint start{};
    const AerialTargetTrackSnapshot acquiring{
        .phase = AerialTargetTrackPhase::acquiring,
        .consecutive_observations = 2,
        .required_observations = 3,
        .accepted_observations = 2,
        .observation_age_ms = 0.0,
        .confidence_percent = 70.0,
        .horizontal_error = std::nullopt,
        .center_y_ratio = 0.4,
        .width_ratio = 0.1,
        .height_ratio = 0.1,
    };

    require(
        runtime
            .update(vehicle, startup, failsafe, start, std::nullopt, acquiring)
            .empty(),
        "an unconfirmed target must not produce a yaw command");

    const AerialTargetTrackSnapshot locked_right{
        .phase = AerialTargetTrackPhase::tracking,
        .consecutive_observations = 3,
        .required_observations = 3,
        .accepted_observations = 3,
        .observation_age_ms = 0.0,
        .confidence_percent = 80.0,
        .horizontal_error = 0.6,
        .center_y_ratio = 0.4,
        .width_ratio = 0.1,
        .height_ratio = 0.1,
    };
    const auto yaw = only_action(runtime.update(vehicle,
                                     startup,
                                     failsafe,
                                     start + std::chrono::milliseconds(10),
                                     std::nullopt,
                                     locked_right),
        FlightAction::yaw_rate,
        "stable target lock");
    require(yaw.yaw_rate_degrees_per_second > 68.0 &&
                yaw.yaw_rate_degrees_per_second < 69.0,
        "yaw guidance must derive a signed rate from target error");
    require(runtime
                .update(vehicle,
                    startup,
                    failsafe,
                    start + std::chrono::milliseconds(100),
                    std::nullopt,
                    locked_right)
                .empty(),
        "yaw commands must be rate limited");

    const AerialTargetTrackSnapshot lost{
        .phase = AerialTargetTrackPhase::searching,
        .consecutive_observations = 0,
        .required_observations = 3,
        .accepted_observations = 0,
        .observation_age_ms = std::nullopt,
        .confidence_percent = std::nullopt,
        .horizontal_error = std::nullopt,
        .center_y_ratio = std::nullopt,
        .width_ratio = std::nullopt,
        .height_ratio = std::nullopt,
    };
    const auto hold = only_action(runtime.update(vehicle,
                                      startup,
                                      failsafe,
                                      start + std::chrono::milliseconds(600),
                                      std::nullopt,
                                      lost),
        FlightAction::yaw_rate,
        "target loss yaw hold");
    require(hold.yaw_rate_degrees_per_second == 0.0 &&
                runtime.snapshot().detail == "TARGET SEARCHING | GUIDED HOLD",
        "target loss must cancel the pending turn and preserve heading");
}

void runtime_streams_fresh_target_and_lands() {
    AutonomyRuntime runtime{{.enabled = true}};
    auto vehicle = flying_vehicle();
    const auto startup = completed_startup();
    const TimePoint start{};
    const BodyFramePosition target{
        .forward_m = 0.4,
        .right_m = -0.2,
        .down_m = 8.1,
    };

    auto action = only_action(
        runtime.update(vehicle, startup, accepted_failsafe(), start, target),
        FlightAction::landing_target,
        "fresh target must start LANDING_TARGET stream");
    require(action.x_m == target.forward_m && action.y_m == target.right_m &&
                action.z_m == target.down_m,
        "guidance must preserve body-FRD target");
    runtime.on_action_sent(action, true, start);

    require(runtime
                .update(vehicle,
                    startup,
                    accepted_failsafe(),
                    start + std::chrono::milliseconds(400))
                .empty(),
        "target loss must suppress guidance immediately");
    require(!runtime.snapshot().vision_landing_target_active,
        "target loss must be observable");

    action = only_action(runtime.update(vehicle,
                             startup,
                             accepted_failsafe(),
                             start + std::chrono::milliseconds(500),
                             target),
        FlightAction::landing_target,
        "reacquisition must restart target stream");
    runtime.on_action_sent(action,
        true,
        start + std::chrono::milliseconds(500));

    const auto actions = runtime.update(vehicle,
        startup,
        accepted_failsafe(),
        start + std::chrono::milliseconds(1500),
        target);
    const auto land = std::find_if(actions.begin(),
        actions.end(),
        [](const FlightActionRequest& request) {
            return request.action == FlightAction::land;
        });
    require(land != actions.end(),
        "one second of continuous target must request LAND");
    runtime.on_action_sent(*land,
        true,
        start + std::chrono::milliseconds(1500));
    runtime.on_command_ack(FlightAction::land,
        FlightCommandAckOutcome::accepted,
        0,
        1,
        start + std::chrono::milliseconds(1500));

    vehicle.relative_altitude_m = 0.15;
    require(runtime
                .update(vehicle,
                    startup,
                    accepted_failsafe(),
                    start + std::chrono::seconds(2),
                    target)
                .empty(),
        "touchdown must stop target output");
    vehicle.armed = false;
    static_cast<void>(runtime.update(vehicle,
        startup,
        accepted_failsafe(),
        start + std::chrono::seconds(3),
        target));
    require(runtime.snapshot().phase == AutonomyRuntimePhase::completed,
        "runtime completes only after disarm");
}

void runtime_streams_target_at_ten_hz() {
    AutonomyRuntime runtime{{.enabled = true}};
    const auto vehicle = flying_vehicle();
    const auto startup = completed_startup();
    const auto failsafe = accepted_failsafe();
    const TimePoint start{};
    const BodyFramePosition target{
        .forward_m = 0.1,
        .right_m = 0.2,
        .down_m = 8.0,
    };

    only_action(runtime.update(vehicle, startup, failsafe, start, target),
        FlightAction::landing_target,
        "fresh target must be sent immediately");
    require(runtime
                .update(vehicle,
                    startup,
                    failsafe,
                    start + std::chrono::milliseconds(99),
                    target)
                .empty(),
        "target stream must not exceed ten hertz");
    only_action(runtime.update(vehicle,
                    startup,
                    failsafe,
                    start + std::chrono::milliseconds(100),
                    target),
        FlightAction::landing_target,
        "target stream must emit at ten hertz");
}

void terminal_descent_requires_stable_low_alignment() {
    AutonomyRuntime runtime{{.enabled = true}};
    auto vehicle = flying_vehicle();
    const auto startup = completed_startup();
    const auto failsafe = accepted_failsafe();
    const TimePoint start{};
    const BodyFramePosition centered_target{
        .forward_m = 0.05,
        .right_m = -0.05,
        .down_m = 1.5,
    };

    enter_landing(runtime, vehicle, startup, failsafe, start, centered_target);

    vehicle.relative_altitude_m = 1.4;
    static_cast<void>(runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::milliseconds(1100),
        centered_target));
    static_cast<void>(runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::milliseconds(1600),
        centered_target));
    require(runtime
                .update(vehicle,
                    startup,
                    failsafe,
                    start + std::chrono::milliseconds(1700))
                .empty(),
        "confirmed low alignment must hand off without another command");
    require(runtime.snapshot().terminal_descent_active,
        "terminal descent handoff must be observable");
    require(runtime.snapshot().detail ==
                "TARGET OUT OF VIEW - TERMINAL DESCENT CONTINUES",
        "terminal descent must explain expected target loss");
    require(runtime
                .update(vehicle,
                    startup,
                    failsafe,
                    start + std::chrono::milliseconds(1800),
                    centered_target)
                .empty(),
        "terminal descent must ignore close-range target reacquisition");
}

void target_loss_without_alignment_is_not_terminal_handoff() {
    AutonomyRuntime runtime{{.enabled = true}};
    auto vehicle = flying_vehicle();
    const auto startup = completed_startup();
    const auto failsafe = accepted_failsafe();
    const TimePoint start{};
    const BodyFramePosition off_center_target{
        .forward_m = 0.5,
        .right_m = 0.0,
        .down_m = 1.5,
    };

    enter_landing(runtime,
        vehicle,
        startup,
        failsafe,
        start,
        off_center_target);

    vehicle.relative_altitude_m = 1.4;
    static_cast<void>(runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::milliseconds(1600),
        off_center_target));
    static_cast<void>(runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::milliseconds(1700)));
    require(!runtime.snapshot().terminal_descent_active,
        "off-center target loss must not claim a safe terminal handoff");

    const BodyFramePosition centered_target{
        .forward_m = 0.05,
        .right_m = 0.0,
        .down_m = 1.5,
    };
    static_cast<void>(runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::milliseconds(1800),
        centered_target));
    static_cast<void>(runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::milliseconds(2000)));
    static_cast<void>(runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::milliseconds(2100),
        centered_target));
    static_cast<void>(runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::milliseconds(2500)));
    require(!runtime.snapshot().terminal_descent_active,
        "an interrupted alignment window must restart its dwell timer");
}

void prolonged_target_loss_requests_fallback_land() {
    AutonomyRuntime runtime{{
        .enabled = true,
        .target_loss_land_after = std::chrono::seconds(5),
    }};
    const auto vehicle = flying_vehicle();
    const auto startup = completed_startup();
    const TimePoint start{};

    require(
        runtime.update(vehicle, startup, accepted_failsafe(), start).empty(),
        "initial target absence must allow a bounded search window");
    const auto land = only_action(runtime.update(vehicle,
                                      startup,
                                      accepted_failsafe(),
                                      start + std::chrono::seconds(5)),
        FlightAction::land,
        "prolonged target loss must request fallback LAND");
    require(land.vehicle_system_id == 1,
        "fallback LAND must target the active flight controller");
}

void link_loss_stops_runtime_output() {
    AutonomyRuntime runtime{{.enabled = true}};
    auto vehicle = flying_vehicle();
    vehicle.connected = false;

    require(runtime
                .update(vehicle,
                    completed_startup(),
                    accepted_failsafe(),
                    TimePoint{},
                    BodyFramePosition{0.0, 0.0, 8.0})
                .empty(),
        "link loss must emit no motion command");
    require(runtime.snapshot().phase == AutonomyRuntimePhase::failed,
        "link loss must fail the companion runtime");
}

void rejected_link_failsafe_stops_runtime_output() {
    AutonomyRuntime runtime{{.enabled = true}};
    CompanionLinkFailsafeSnapshot rejected;
    rejected.phase = CompanionLinkFailsafePhase::rejected;
    rejected.detail = "FS_OPTIONS bypasses the GCS failsafe";

    require(runtime
                .update(flying_vehicle(),
                    completed_startup(),
                    rejected,
                    TimePoint{},
                    BodyFramePosition{0.0, 0.0, 8.0})
                .empty(),
        "invalid link failsafe must suppress autonomy output");
    require(runtime.snapshot().phase == AutonomyRuntimePhase::failed,
        "invalid link failsafe must fail the autonomy runtime");
}

void restart_clears_terminal_autonomy_state() {
    AutonomyRuntime runtime{{.enabled = true}};
    auto vehicle = flying_vehicle();
    vehicle.connected = false;
    static_cast<void>(runtime.update(vehicle,
        completed_startup(),
        accepted_failsafe(),
        TimePoint{}));
    require(runtime.snapshot().phase == AutonomyRuntimePhase::failed,
        "test setup must reach a terminal autonomy state");

    runtime.restart();

    const auto snapshot = runtime.snapshot();
    require(snapshot.phase == AutonomyRuntimePhase::waiting_for_startup &&
                snapshot.land_attempt == 0 &&
                !snapshot.vision_landing_target_active &&
                !snapshot.failure_result.has_value(),
        "restart must reset autonomy state for another flight");
}

void operator_controls_runtime_mode_and_rtl_lifecycle() {
    AutonomyRuntime runtime{{
        .enabled = true,
        .start_automatically = false,
    }};
    const TimePoint start{};

    require(runtime.snapshot().phase == AutonomyRuntimePhase::idle &&
                runtime
                    .update(flying_vehicle(),
                        completed_startup(),
                        accepted_failsafe(),
                        start)
                    .empty(),
        "operator-controlled autonomy must stay idle without a mission");

    runtime.restart(AutonomyRuntimeMode::aerial_observation);
    require(runtime.snapshot().phase ==
                    AutonomyRuntimePhase::waiting_for_startup &&
                runtime.snapshot().detail.find("aerial observation") !=
                    std::string::npos,
        "operator must be able to select the aerial tracking mode");

    runtime.begin_return_to_launch(1, start);
    runtime.on_command_ack(FlightAction::return_to_launch,
        FlightCommandAckOutcome::accepted,
        0,
        1,
        start);
    const auto rtl = only_action(runtime.update(flying_vehicle(),
                                     completed_startup(),
                                     accepted_failsafe(),
                                     start),
        FlightAction::return_to_launch,
        "a stale ACK must not suppress the new operator RTL command");
    runtime.on_action_sent(rtl, true, start);
    runtime.on_command_ack(FlightAction::return_to_launch,
        FlightCommandAckOutcome::accepted,
        0,
        1,
        start);
    require(runtime.snapshot().phase ==
                    AutonomyRuntimePhase::returning_to_launch &&
                runtime.snapshot().detail.find("accepted") != std::string::npos,
        "accepted RTL must remain active while the vehicle is armed");

    auto disarmed = flying_vehicle();
    disarmed.armed = false;
    require(runtime.update(disarmed,
                       completed_startup(),
                       accepted_failsafe(),
                       start + std::chrono::seconds(1))
                    .empty() &&
                runtime.snapshot().phase == AutonomyRuntimePhase::completed,
        "RTL must complete after the flight controller reports disarm");
}

void rtl_retries_and_fails_without_an_acknowledgement() {
    AutonomyRuntime runtime{{
        .enabled = true,
        .start_automatically = false,
    }};
    const auto vehicle = flying_vehicle();
    const auto startup = completed_startup();
    const auto failsafe = accepted_failsafe();
    const TimePoint start{};
    runtime.begin_return_to_launch(1, start);

    for (std::size_t attempt = 0; attempt < 3; ++attempt) {
        const auto now = start + std::chrono::seconds(attempt * 3);
        const auto rtl =
            only_action(runtime.update(vehicle, startup, failsafe, now),
                FlightAction::return_to_launch,
                "missing RTL acknowledgement must trigger a bounded retry");
        require(rtl.confirmation == attempt,
            "RTL retries must increase MAVLink confirmation");
        runtime.on_action_sent(rtl, true, now);
    }

    require(runtime.update(vehicle,
                       startup,
                       failsafe,
                       start + std::chrono::seconds(9))
                    .empty() &&
                runtime.snapshot().phase == AutonomyRuntimePhase::failed,
        "RTL must fail visibly after acknowledgement retry exhaustion");
}

} // namespace

void run_autonomy_runtime_tests() {
    runtime_waits_for_startup();
    aerial_observation_holds_after_takeoff_without_landing();
    aerial_observation_yaws_only_for_a_stable_target_lock();
    runtime_streams_fresh_target_and_lands();
    runtime_streams_target_at_ten_hz();
    terminal_descent_requires_stable_low_alignment();
    target_loss_without_alignment_is_not_terminal_handoff();
    prolonged_target_loss_requests_fallback_land();
    link_loss_stops_runtime_output();
    rejected_link_failsafe_stops_runtime_output();
    restart_clears_terminal_autonomy_state();
    operator_controls_runtime_mode_and_rtl_lifecycle();
    rtl_retries_and_fails_without_an_acknowledgement();
}
