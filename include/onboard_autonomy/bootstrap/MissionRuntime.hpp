#pragma once

#include "onboard_autonomy/mission/EnvironmentProfile.hpp"
#include "onboard_autonomy/mission/autonomy/AutonomyRuntime.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <variant>

namespace onboard_autonomy::mission {
class CompanionApplication;
}

namespace onboard_autonomy::mission::ports {
class Transport;
}

namespace onboard_autonomy::bootstrap {

enum class MissionEnvironment {
    hardware,
    simulation,
};

struct UdpMissionConnection {
    std::string bind_address;
    std::uint16_t port{};
};

struct SerialMissionConnection {
    std::string device;
    std::uint32_t baud_rate{};
};

using MissionConnection =
    std::variant<UdpMissionConnection, SerialMissionConnection>;

struct RpicamMissionSource {
    std::uint32_t frames_per_second{};
};

struct GStreamerMissionSource {
    std::uint16_t udp_port{};
};

using MissionCameraSource =
    std::variant<RpicamMissionSource, GStreamerMissionSource>;

struct AprilTagMissionConfig {
    std::string calibration_file;
    std::string extrinsics_file;
    std::optional<double> tag_size_m;
};

struct MissionCameraConfig {
    MissionCameraSource source;
    std::optional<AprilTagMissionConfig> apriltag;
    std::uint32_t frame_width{};
    std::uint32_t frame_height{};
};

struct MissionRuntimeConfig {
    MissionConnection connection;
    std::optional<MissionCameraConfig> camera;
    std::optional<mission::SimulatedWindProfile> simulated_wind;
    MissionEnvironment environment{MissionEnvironment::hardware};
    bool autonomous{};
    mission::AutonomyRuntimeMode autonomy_mode{
        mission::AutonomyRuntimeMode::precision_landing};
    bool motion_commands_requested{};
};

// Owns only the adapters and application state required to execute a mission.
class MissionRuntime {
  public:
    explicit MissionRuntime(const MissionRuntimeConfig& config);
    ~MissionRuntime();

    MissionRuntime(const MissionRuntime&) = delete;
    MissionRuntime& operator=(const MissionRuntime&) = delete;
    MissionRuntime(MissionRuntime&&) = delete;
    MissionRuntime& operator=(MissionRuntime&&) = delete;

    [[nodiscard]] mission::CompanionApplication& application();
    [[nodiscard]] mission::ports::Transport& transport();

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard_autonomy::bootstrap
