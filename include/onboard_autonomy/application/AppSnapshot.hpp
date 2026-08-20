#pragma once

#include "onboard_autonomy/application/AutonomyRuntime.hpp"
#include "onboard_autonomy/application/CameraMonitor.hpp"
#include "onboard_autonomy/application/CompanionLinkFailsafe.hpp"
#include "onboard_autonomy/application/EnvironmentProfile.hpp"
#include "onboard_autonomy/application/FlightStartupController.hpp"
#include "onboard_autonomy/domain/VehicleState.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::application {

enum class LinkEventDirection {
    outbound,
    inbound,
};

enum class LinkEventStatus {
    neutral,
    pending,
    success,
    warning,
    failure,
};

struct LinkEvent {
    std::uint64_t sequence{0};
    std::chrono::milliseconds elapsed{};
    LinkEventDirection direction{LinkEventDirection::outbound};
    LinkEventStatus status{LinkEventStatus::neutral};
    std::string label;
    std::string detail;
};

struct LinkActivity {
    std::uint64_t sequence{0};
    std::chrono::milliseconds observed_at{};
    std::string message_name;
    std::string detail;
};

enum class TelemetrySetupState {
    waiting_for_vehicle,
    configuring,
    active,
    failed,
};

struct TelemetryStatus {
    TelemetrySetupState state{TelemetrySetupState::waiting_for_vehicle};
    std::size_t completed_requests{0};
    std::size_t total_requests{0};
    std::string current_stream;
    std::size_t attempt{0};
    std::optional<std::uint8_t> failure_result;
};

struct AppSnapshot {
    domain::VehicleSnapshot vehicle;
    bool companion_heartbeat_active{false};
    CompanionLinkFailsafeSnapshot companion_link_failsafe;
    TelemetryStatus telemetry;
    std::optional<SimulatedWindProfile> simulated_wind;
    std::optional<CameraSnapshot> camera;
    std::optional<VisionSnapshot> vision;
    FlightStartupSnapshot flight_startup;
    AutonomyRuntimeSnapshot autonomy;
    bool motion_commands_allowed{false};
    std::vector<LinkEvent> link_events;
    std::chrono::milliseconds elapsed{};
    std::optional<LinkActivity> tx_activity;
    std::optional<LinkActivity> rx_activity;
};

} // namespace onboard_autonomy::application
