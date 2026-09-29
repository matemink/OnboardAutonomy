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
    require(snapshot.navigation_ready,
        "GPS should provide the initial navigation estimate");
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

    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, now + std::chrono::seconds(3));
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
    require(!snapshot.navigation_ready,
        "missing position sources must not provide navigation readiness");
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

void battery_fields_expire_independently() {
    using namespace std::chrono_literals;
    onboard_autonomy::mission::VehicleState state;
    const onboard_autonomy::mission::TimePoint start{};
    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, start);
    configure_battery_threshold(state, start);
    state.on_battery(15.2, 0.4, 88, start);
    for (int second = 1; second <= 11; ++second) {
        const auto now = start + std::chrono::seconds{second};
        state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, now);
        state.on_gps(3, 12, now);
        state.on_system_status(kHealthySensorFlags, kHealthySensorFlags,
            std::nullopt, 0.5, std::nullopt, now);
    }
    const auto expired = state.snapshot(start + 11s);
    require(expired.system_health_ok && expired.gps_ready &&
            expired.battery_current_a == 0.5,
        "other live fields must still be observable");
    require(!expired.battery_voltage_v && !expired.battery_remaining_pct &&
            !expired.battery_ready && !expired.armable,
        "current-only samples must not keep old voltage or charge alive");
    state.on_battery(15.0, std::nullopt, std::nullopt, start + 11s);
    const auto refreshed = state.snapshot(start + 11s);
    require(refreshed.battery_voltage_v == 15.0 &&
            refreshed.battery_arming_voltage_v == 10.5 &&
            !refreshed.battery_remaining_pct,
        "new voltage must not resurrect the expired percentage");
}

void unknown_battery_samples_do_not_refresh_measurements() {
    onboard_autonomy::mission::VehicleState state;
    const onboard_autonomy::mission::TimePoint start{};
    state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, start);
    state.on_battery(15.2, 0.4, 88, start);
    for (int second = 1; second <= 11; ++second) {
        const auto now = start + std::chrono::seconds{second};
        state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, now);
        state.on_battery(std::nullopt, std::nullopt, std::nullopt, now);
    }
    const auto snapshot = state.snapshot(start + std::chrono::seconds{11});
    require(!snapshot.battery_voltage_v && !snapshot.battery_current_a &&
            !snapshot.battery_remaining_pct,
        "unknown battery reports must not extend measurement freshness");
}

void controller_change_or_reconnect_discards_previous_telemetry() {
    for (const bool change_controller : {false, true}) {
        onboard_autonomy::mission::VehicleState state;
        const onboard_autonomy::mission::TimePoint start{};
        state.on_heartbeat(1, 1, 2, 3, 0, 0, 3, start);
        configure_battery_threshold(state, start);
        state.on_system_status(kHealthySensorFlags, kHealthySensorFlags,
            15.2, 0.4, 88, start);
        state.on_gps(3, 12, start);
        state.on_global_position(2500, start);
        state.on_local_position(1.0F, 2.0F, 3.0F, start);
        state.on_attitude(0.1F, 0.2F, 0.3F, 0.0F, start);
        state.on_autopilot_metadata({});
        state.on_status_text(4, "old controller warning", start);
        const auto now = start + std::chrono::seconds{change_controller ? 1 : 4};
        state.on_heartbeat(change_controller ? 2 : 1, 1, 2, 3, 0, 0, 3, now);
        const auto snapshot = state.snapshot(now);
        require(snapshot.connected && !snapshot.gps_fix_type &&
                !snapshot.battery_voltage_v && !snapshot.battery_current_a &&
                !snapshot.battery_remaining_pct && !snapshot.system_health_known &&
                !snapshot.relative_altitude_m && !snapshot.local_north_m &&
                !snapshot.yaw_rad && !snapshot.autopilot_metadata &&
                snapshot.warnings.empty(),
            "a new connection must not inherit previous-session telemetry");
        state.on_battery(15.2, std::nullopt, std::nullopt, now);
        require(!state.snapshot(now).battery_arming_voltage_v,
            "new voltage must not resurrect the previous arming threshold");
    }
}

} // namespace

void run_vehicle_state_tests() {
    battery_fields_expire_independently();
    unknown_battery_samples_do_not_refresh_measurements();
    controller_change_or_reconnect_discards_previous_telemetry();
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
