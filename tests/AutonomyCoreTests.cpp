#include "TestCases.hpp"

#include "onboard_autonomy/mission/autonomy/DecisionEngine.hpp"
#include "onboard_autonomy/mission/safety/SafetySupervisor.hpp"

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using onboard_autonomy::mission::BodyFramePosition;
using onboard_autonomy::mission::DecisionEngine;
using onboard_autonomy::mission::MotionSafetyStatus;
using onboard_autonomy::mission::SafetySupervisor;
using onboard_autonomy::mission::TimePoint;
using onboard_autonomy::mission::WorldState;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

WorldState tracking_world(const TimePoint now) {
    return {
        .flight_controller_connected = true,
        .vehicle_armed = true,
        .vehicle_system_id = 1,
        .landing_target =
            BodyFramePosition{
                .forward_m = 0.4,
                .right_m = -0.2,
                .down_m = 8.0,
            },
        .observed_at = now,
    };
}

void decision_engine_requires_a_current_target() {
    const DecisionEngine engine;
    auto world = tracking_world(TimePoint{});
    world.landing_target.reset();
    require(!engine.decide(world).has_value(),
        "decision engine must not fabricate a target");

    world = tracking_world(TimePoint{});
    const auto desired = engine.decide(world);
    require(desired.has_value() && desired->landing_target.forward_m == 0.4 &&
                desired->landing_target.right_m == -0.2 &&
                desired->landing_target.down_m == 8.0,
        "decision engine must preserve the observed body-FRD target");
    require(desired->valid_until - desired->created_at ==
                std::chrono::milliseconds(250),
        "desired motion must have a bounded lifetime");
}

void safety_supervisor_rejects_degraded_world_state() {
    const DecisionEngine engine;
    const SafetySupervisor supervisor;
    const TimePoint start{};
    auto world = tracking_world(start);
    const auto desired = engine.decide(world);

    require(supervisor.supervise(world, desired, start).status ==
                MotionSafetyStatus::allowed,
        "fresh armed connected intent must be allowed");

    world.flight_controller_connected = false;
    require(supervisor.supervise(world, desired, start).status ==
                MotionSafetyStatus::flight_controller_disconnected,
        "link loss must suppress desired motion");

    world = tracking_world(start);
    world.vehicle_armed = false;
    require(supervisor.supervise(world, desired, start).status ==
                MotionSafetyStatus::vehicle_disarmed,
        "disarmed state must suppress desired motion");

    world = tracking_world(start);
    require(supervisor
                    .supervise(world,
                        desired,
                        start + std::chrono::milliseconds(251))
                    .status == MotionSafetyStatus::stale_intent,
        "expired desired motion must never reach guidance");
}

void safety_supervisor_rejects_non_finite_target() {
    const DecisionEngine engine;
    const SafetySupervisor supervisor;
    const TimePoint start{};
    auto world = tracking_world(start);
    world.landing_target->forward_m = std::numeric_limits<double>::quiet_NaN();

    require(supervisor.supervise(world, engine.decide(world), start).status ==
                MotionSafetyStatus::invalid_target,
        "non-finite vision target must be rejected");
}

} // namespace

void run_autonomy_core_tests() {
    decision_engine_requires_a_current_target();
    safety_supervisor_rejects_degraded_world_state();
    safety_supervisor_rejects_non_finite_target();
}
