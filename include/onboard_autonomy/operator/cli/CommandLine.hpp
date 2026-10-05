#pragma once

#include "onboard_autonomy/mission/EnvironmentProfile.hpp"
#include "onboard_autonomy/operator/cli/CommandLineDefaults.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace onboard_autonomy::operator_interface::cli {

struct UdpConnectionOptions {
    std::string bind_address{defaults::kUdpBindAddress};
    std::uint16_t port{defaults::kMavlinkUdpPort};
};

struct SerialConnectionOptions {
    std::string device;
    std::uint32_t baud_rate{defaults::kSerialBaudRate};
};

using MavlinkConnectionOptions =
    std::variant<UdpConnectionOptions, SerialConnectionOptions>;

struct RpicamOptions {
    std::uint32_t frames_per_second{defaults::kCameraFramesPerSecond};
};

struct GStreamerCameraOptions {
    std::uint16_t udp_port{defaults::kCameraUdpPort};
};

using CameraSourceOptions = std::variant<RpicamOptions, GStreamerCameraOptions>;

struct CameraPreviewOptions {
    std::uint16_t port{defaults::kCameraPreviewPort};
};

struct ForwardCameraOptions {
    std::uint16_t udp_port{defaults::kForwardCameraUdpPort};
    std::uint32_t frame_width{defaults::kCameraFrameWidth};
    std::uint32_t frame_height{defaults::kCameraFrameHeight};
};

struct CameraOptions {
    CameraSourceOptions source{RpicamOptions{}};
    std::uint32_t frame_width{defaults::kCameraFrameWidth};
    std::uint32_t frame_height{defaults::kCameraFrameHeight};
};

struct OperatorInterfaceOptions {
    bool interactive{};
    bool json_output{};
    std::uint32_t snapshot_interval_ms{defaults::kSnapshotIntervalMs};
    std::string board_types_file;
};

struct DiagnosticsOptions {
    std::optional<CameraPreviewOptions> camera_preview;
    std::optional<ForwardCameraOptions> forward_camera;
    std::string log_file;
};

struct HardwareLaunchOptions {
    MavlinkConnectionOptions connection{UdpConnectionOptions{}};
    std::optional<CameraOptions> camera;
    OperatorInterfaceOptions operator_interface;
    DiagnosticsOptions diagnostics;
};

struct SimulationLaunchOptions {
    UdpConnectionOptions connection;
    std::optional<CameraOptions> camera;
    std::optional<mission::SimulatedWindProfile> wind;
    OperatorInterfaceOptions operator_interface;
    DiagnosticsOptions diagnostics;
};

using CommandLineOptions =
    std::variant<HardwareLaunchOptions, SimulationLaunchOptions>;

[[nodiscard]] CommandLineOptions parse_command_line(
    const std::vector<std::string_view>& arguments);

[[nodiscard]] std::string_view command_line_help();

} // namespace onboard_autonomy::operator_interface::cli
