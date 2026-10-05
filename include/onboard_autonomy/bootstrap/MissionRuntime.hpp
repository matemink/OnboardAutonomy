#pragma once

#include "onboard_autonomy/mission/EnvironmentProfile.hpp"

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
    std::string sensor_mode{"2304:1296:10:P"};
};

struct GStreamerMissionSource {
    std::uint16_t udp_port{};
};

using MissionCameraSource =
    std::variant<RpicamMissionSource, GStreamerMissionSource>;

struct MissionCameraConfig {
    MissionCameraSource source;
    std::uint32_t frame_width{};
    std::uint32_t frame_height{};
};

struct MissionRuntimeConfig {
    MissionConnection connection;
    std::optional<MissionCameraConfig> camera;
    std::optional<mission::SimulatedWindProfile> simulated_wind;
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
