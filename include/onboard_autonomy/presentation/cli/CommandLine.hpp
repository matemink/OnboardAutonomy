#pragma once

#include "onboard_autonomy/application/EnvironmentProfile.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace onboard_autonomy::presentation::cli {

enum class TransportBackend {
    udp,
    serial,
};

enum class CameraBackend {
    rpicam,
    gstreamer,
};

struct CommandLineOptions {
    TransportBackend transport{TransportBackend::udp};
    std::string udp_bind{"0.0.0.0"};
    std::uint16_t udp_port{14550};
    std::string serial_device;
    std::uint32_t baud_rate{115200};
    std::uint32_t snapshot_interval_ms{1000};
    bool camera_enabled{false};
    CameraBackend camera_backend{CameraBackend::rpicam};
    std::uint16_t camera_udp_port{5601};
    bool apriltag_enabled{false};
    std::string camera_calibration_file;
    std::string camera_extrinsics_file;
    std::optional<double> apriltag_tag_size_m;
    bool camera_preview_enabled{false};
    std::uint16_t camera_preview_port{8080};
    std::uint32_t camera_width{640};
    std::uint32_t camera_height{480};
    std::uint32_t camera_fps{30};
    std::string board_types_file;
    bool json_output{false};
    bool sitl_mode{false};
    std::optional<application::SimulatedWindProfile> simulated_wind;
    bool autonomous{false};
    bool exit_after_autonomy{false};
    bool interactive{false};
    bool show_help{false};
};

[[nodiscard]] CommandLineOptions parse_command_line(
    const std::vector<std::string_view>& arguments
);

[[nodiscard]] std::string command_line_help();

}  // namespace onboard_autonomy::presentation::cli
