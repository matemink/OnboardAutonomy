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
        "missing forward detections must not trigger LAND in observation mode");
    const auto snapshot = runtime.snapshot();
    require(snapshot.phase == AutonomyRuntimePhase::active &&
                snapshot.detail == "TARGET SEARCHING | GUIDED HOLD",
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
            .update(vehicle, startup, failsafe, start, acquiring)
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
                                      lost),
        FlightAction::yaw_rate,
        "target loss yaw hold");
    require(hold.yaw_rate_degrees_per_second == 0.0 &&
                runtime.snapshot().detail == "TARGET SEARCHING | GUIDED HOLD",
        "target loss must cancel the pending turn and preserve heading");
}

void aerial_tracking_recovers_after_a_transient_link_loss() {
    AutonomyRuntime runtime{{
        .enabled = true,
        .mode = AutonomyRuntimeMode::aerial_observation,
        .aerial_link_loss_grace = std::chrono::seconds(1),
        .aerial_link_recovery_hold = std::chrono::milliseconds(200),
    }};
    auto vehicle = flying_vehicle();
    const auto startup = completed_startup();
    const auto failsafe = accepted_failsafe();
    const TimePoint start{};

    static_cast<void>(runtime.update(vehicle, startup, failsafe, start));
    const AerialTargetTrackSnapshot initial_target{
        .phase = AerialTargetTrackPhase::tracking,
        .consecutive_observations = 3,
        .required_observations = 3,
        .accepted_observations = 9,
        .observation_age_ms = 0.0,
        .confidence_percent = 80.0,
        .horizontal_error = 0.4,
        .center_y_ratio = 0.4,
        .width_ratio = 0.1,
        .height_ratio = 0.1,
    };
    static_cast<void>(runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::milliseconds(10),
        initial_target));
    vehicle.connected = false;
    const auto disconnected_stop =
        only_action(runtime.update(vehicle,
                        startup,
                        failsafe,
                        start + std::chrono::milliseconds(100)),
            FlightAction::yaw_rate,
            "transient link loss");
    require(disconnected_stop.yaw_rate_degrees_per_second == 0.0,
        "transient link loss must attempt to stop active yaw");
    require(runtime.snapshot().phase == AutonomyRuntimePhase::suspended,
        "transient link loss must suspend aerial tracking");

    vehicle.connected = true;
    CompanionLinkFailsafeSnapshot refreshing_failsafe;
    refreshing_failsafe.phase = CompanionLinkFailsafePhase::reading_parameters;
    const auto recovered_stop =
        only_action(runtime.update(vehicle,
                        startup,
                        refreshing_failsafe,
                        start + std::chrono::milliseconds(200)),
            FlightAction::yaw_rate,
            "recovered controller link");
    require(recovered_stop.yaw_rate_degrees_per_second == 0.0,
        "the first command after link recovery must stop stale yaw");
    require(runtime.snapshot().phase == AutonomyRuntimePhase::suspended,
        "tracking must wait for the companion-link policy to be revalidated");

    static_cast<void>(runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::milliseconds(450)));
    require(runtime.snapshot().phase == AutonomyRuntimePhase::active,
        "tracking must resume after stable link and failsafe recovery");

    const AerialTargetTrackSnapshot recovered_target{
        .phase = AerialTargetTrackPhase::tracking,
        .consecutive_observations = 3,
        .required_observations = 3,
        .accepted_observations = 10,
        .observation_age_ms = 0.0,
        .confidence_percent = 80.0,
        .horizontal_error = 0.4,
        .center_y_ratio = 0.4,
        .width_ratio = 0.1,
        .height_ratio = 0.1,
    };
    const auto resumed_yaw =
        only_action(runtime.update(vehicle,
                        startup,
                        failsafe,
                        start + std::chrono::milliseconds(460),
                        recovered_target),
            FlightAction::yaw_rate,
            "recovered target lock");
    require(resumed_yaw.yaw_rate_degrees_per_second > 0.0,
        "recovered tracking must emit fresh yaw guidance");
}

void prolonged_aerial_link_loss_fails_tracking() {
    AutonomyRuntime runtime{{
        .enabled = true,
        .mode = AutonomyRuntimeMode::aerial_observation,
        .aerial_link_loss_grace = std::chrono::milliseconds(500),
    }};
    auto vehicle = flying_vehicle();
    const auto startup = completed_startup();
    const auto failsafe = accepted_failsafe();
    const TimePoint start{};

    static_cast<void>(runtime.update(vehicle, startup, failsafe, start));
    vehicle.connected = false;
    static_cast<void>(runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::milliseconds(100)));
    static_cast<void>(runtime.update(vehicle,
        startup,
        failsafe,
        start + std::chrono::milliseconds(600)));

    require(runtime.snapshot().phase == AutonomyRuntimePhase::failed,
        "confirmed aerial link loss must still fail tracking");
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
                    TimePoint{})
                .empty(),
        "invalid link failsafe must suppress autonomy output");
    require(runtime.snapshot().phase == AutonomyRuntimePhase::failed,
        "invalid link failsafe must fail the autonomy runtime");
}

void restart_clears_terminal_autonomy_state() {
    AutonomyRuntime runtime{{.enabled = true}};
    auto vehicle = flying_vehicle();
    auto failsafe = accepted_failsafe();
    failsafe.phase = CompanionLinkFailsafePhase::rejected;
    static_cast<void>(runtime.update(vehicle,
        completed_startup(),
        failsafe,
        TimePoint{}));
    require(runtime.snapshot().phase == AutonomyRuntimePhase::failed,
        "test setup must reach a terminal autonomy state");

    runtime.restart();

    const auto snapshot = runtime.snapshot();
    require(snapshot.phase == AutonomyRuntimePhase::waiting_for_startup &&
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
    aerial_tracking_recovers_after_a_transient_link_loss();
    prolonged_aerial_link_loss_fails_tracking();
    rejected_link_failsafe_stops_runtime_output();
    restart_clears_terminal_autonomy_state();
    operator_controls_runtime_mode_and_rtl_lifecycle();
    rtl_retries_and_fails_without_an_acknowledgement();
}
