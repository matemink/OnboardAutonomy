#include "TestCases.hpp"

#include "onboard_autonomy/mission/flight/VehicleState.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t kGyroscopeSensorFlag = 1U;
constexpr std::uint32_t kBatterySensorFlag = 1U << 25U;
constexpr std::uint32_t kHealthySensorFlags =
    kGyroscopeSensorFlag | kBatterySensorFlag;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void configure_battery_threshold(onboard_autonomy::mission::VehicleState& state,
    const onboard_autonomy::mission::TimePoint now,
    const double voltage_v = 10.5) {
    state.on_battery_arming_voltage(voltage_v, now);
}

void healthy_vehicle_is_armable() {
    onboard_autonomy::mission::VehicleState state;
    const onboard_autonomy::mission::TimePoint now{};

    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, now);
    state.on_gps(3, 12, now);
    configure_battery_threshold(state, now);
    state.on_system_status(kHealthySensorFlags,
        kHealthySensorFlags,
        15.2,
        0.4,
        88,
        now);

    const auto snapshot = state.snapshot(now);
    require(snapshot.connected, "heartbeat should establish connection");
    require(snapshot.gps_ready, "3D GPS fix should be ready");
    require(snapshot.battery_ready, "healthy battery should be ready");
    require(snapshot.system_health_ok, "healthy sensors should be ready");
    require(snapshot.armable, "healthy vehicle should be armable");
}

void prearm_warning_blocks_readiness() {
    onboard_autonomy::mission::VehicleState state;
    const onboard_autonomy::mission::TimePoint now{};

    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, now);
    state.on_gps(3, 12, now);
    configure_battery_threshold(state, now);
    state.on_system_status(kHealthySensorFlags,
        kHealthySensorFlags,
        15.2,
        0.4,
        88,
        now);
    state.on_status_text(6, "PreArm: Compass not calibrated", now);

    const auto snapshot = state.snapshot(now);
    require(snapshot.warnings.size() == 1,
        "PreArm text must be retained regardless of severity");
    require(!snapshot.armable, "PreArm warning must block readiness");
}

void quiet_prearm_warning_stops_blocking_but_remains_visible() {
    onboard_autonomy::mission::VehicleState state;
    const onboard_autonomy::mission::TimePoint now{};

    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, now);
    state.on_gps(3, 12, now);
    configure_battery_threshold(state, now);
    state.on_system_status(kHealthySensorFlags,
        kHealthySensorFlags,
        15.2,
        0.4,
        88,
        now);
    state.on_status_text(6, "PreArm: Accels inconsistent", now);

    const auto later = now + std::chrono::seconds(6);
    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, later);
    state.on_gps(3, 12, later);
    state.on_system_status(kHealthySensorFlags,
        kHealthySensorFlags,
        15.2,
        0.4,
        88,
        later);

    const auto snapshot = state.snapshot(later);
    require(snapshot.warnings.size() == 1,
        "quiet warning should remain visible for diagnostics");
    require(snapshot.armable,
        "a warning not repeated for 5 seconds must stop blocking");
}

void stale_heartbeat_disconnects_vehicle() {
    onboard_autonomy::mission::VehicleState state;
    const onboard_autonomy::mission::TimePoint now{};
    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, now);

    const auto later = now + std::chrono::seconds(4);
    const auto snapshot = state.snapshot(later);
    require(!snapshot.connected, "stale heartbeat must disconnect vehicle");
    require(!snapshot.armable, "disconnected vehicle cannot be armable");
}

void missing_data_remains_unknown() {
    onboard_autonomy::mission::VehicleState state;
    const onboard_autonomy::mission::TimePoint now{};
    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, now);

    const auto snapshot = state.snapshot(now);
    require(!snapshot.gps_ready, "missing GPS must not pass");
    require(!snapshot.battery_ready, "missing battery must not pass");
    require(!snapshot.system_health_ok, "missing SYS_STATUS must not pass");
}

void low_battery_blocks_readiness() {
    onboard_autonomy::mission::VehicleState state;
    const onboard_autonomy::mission::TimePoint now{};

    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, now);
    state.on_gps(3, 12, now);
    configure_battery_threshold(state, now);
    state.on_system_status(kHealthySensorFlags,
        kHealthySensorFlags,
        15.2,
        0.4,
        19,
        now);

    const auto snapshot = state.snapshot(now);
    require(!snapshot.battery_ready, "19% battery must not be ready");
    require(!snapshot.armable, "low battery must block readiness");
}

void unhealthy_battery_sensor_blocks_readiness() {
    onboard_autonomy::mission::VehicleState state;
    const onboard_autonomy::mission::TimePoint now{};

    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, now);
    state.on_gps(3, 12, now);
    configure_battery_threshold(state, now);
    state.on_system_status(kHealthySensorFlags,
        kGyroscopeSensorFlag,
        15.2,
        0.4,
        88,
        now);

    const auto snapshot = state.snapshot(now);
    require(!snapshot.battery_ready,
        "unhealthy MAVLink battery sensor must not be ready");
}

void battery_prearm_warning_marks_battery_not_ready() {
    onboard_autonomy::mission::VehicleState state;
    const onboard_autonomy::mission::TimePoint now{};

    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, now);
    state.on_gps(3, 12, now);
    configure_battery_threshold(state, now);
    state.on_system_status(kHealthySensorFlags,
        kHealthySensorFlags,
        15.2,
        0.4,
        88,
        now);
    state.on_status_text(6,
        "PreArm: Battery 1 below minimum arming voltage",
        now);

    const auto snapshot = state.snapshot(now);
    require(!snapshot.battery_ready,
        "battery PreArm warning must mark the battery not ready");
}

void voltage_below_ardupilot_threshold_blocks_readiness() {
    onboard_autonomy::mission::VehicleState state;
    const onboard_autonomy::mission::TimePoint now{};

    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, now);
    state.on_gps(3, 12, now);
    configure_battery_threshold(state, now, 10.0);
    state.on_system_status(kHealthySensorFlags,
        kHealthySensorFlags,
        0.01,
        0.58,
        99,
        now);

    const auto snapshot = state.snapshot(now);
    require(snapshot.battery_arming_voltage_v == 10.0,
        "ArduPilot battery arming threshold must be visible");
    require(!snapshot.battery_ready,
        "voltage below BATT_ARM_VOLT must not be ready");
}

} // namespace

void run_vehicle_state_tests() {
    healthy_vehicle_is_armable();
    prearm_warning_blocks_readiness();
    quiet_prearm_warning_stops_blocking_but_remains_visible();
    stale_heartbeat_disconnects_vehicle();
    missing_data_remains_unknown();
    low_battery_blocks_readiness();
    unhealthy_battery_sensor_blocks_readiness();
    battery_prearm_warning_marks_battery_not_ready();
    voltage_below_ardupilot_threshold_blocks_readiness();
}
