#include "TestCases.hpp"

#include "onboard_autonomy/mission/safety/MotionSafetyPolicy.hpp"

#include <stdexcept>
#include <string>

namespace {

using onboard_autonomy::mission::evaluate_motion_safety;
using onboard_autonomy::mission::MavlinkTransport;
using onboard_autonomy::mission::RuntimeEnvironment;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void unknown_udp_is_observation_only_by_default() {
    const auto passive =
        evaluate_motion_safety(RuntimeEnvironment::hardware_or_unknown,
            MavlinkTransport::udp,
            false);
    require(passive.configuration_valid && !passive.motion_commands_allowed,
        "unknown UDP endpoint must default to observation-only");

    const auto requested =
        evaluate_motion_safety(RuntimeEnvironment::hardware_or_unknown,
            MavlinkTransport::udp,
            true);
    require(!requested.configuration_valid &&
                !requested.motion_commands_allowed,
        "real or unknown hardware over UDP must not receive motion");
}

void explicit_sitl_udp_allows_requested_motion() {
    const auto decision = evaluate_motion_safety(RuntimeEnvironment::sitl,
        MavlinkTransport::udp,
        true);
    require(decision.configuration_valid && decision.motion_commands_allowed,
        "explicit SITL over UDP must allow requested motion");
}

void serial_never_becomes_sitl_motion_transport() {
    const auto physical =
        evaluate_motion_safety(RuntimeEnvironment::hardware_or_unknown,
            MavlinkTransport::serial,
            false);
    require(physical.configuration_valid && !physical.motion_commands_allowed,
        "physical serial observation must remain valid and passive");

    const auto contradictory = evaluate_motion_safety(RuntimeEnvironment::sitl,
        MavlinkTransport::serial,
        true);
    require(!contradictory.configuration_valid &&
                !contradictory.motion_commands_allowed,
        "serial transport must reject explicit SITL motion mode");
}

} // namespace

void run_motion_safety_policy_tests() {
    unknown_udp_is_observation_only_by_default();
    explicit_sitl_udp_allows_requested_motion();
    serial_never_becomes_sitl_motion_transport();
}
