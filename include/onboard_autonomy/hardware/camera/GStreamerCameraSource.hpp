#pragma once

#include "onboard_autonomy/mission/cv/CameraSource.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace onboard_autonomy::hardware::camera {

struct GStreamerCameraConfig {
    static constexpr std::uint32_t kDefaultWidth = 640;
    static constexpr std::uint32_t kDefaultHeight = 480;
    static constexpr std::uint16_t kDefaultUdpPort = 5601;
    static constexpr std::uint32_t kDefaultJitterLatencyMs = 50;
    static constexpr std::uint32_t kDefaultFrameTimeoutMs = 2000;
    static constexpr std::uint32_t kDefaultRestartDelayMs = 500;

    std::uint32_t width{kDefaultWidth};
    std::uint32_t height{kDefaultHeight};
    std::uint16_t udp_port{kDefaultUdpPort};
    std::uint32_t jitter_latency_ms{kDefaultJitterLatencyMs};
    std::uint32_t frame_timeout_ms{kDefaultFrameTimeoutMs};
    std::uint32_t restart_delay_ms{kDefaultRestartDelayMs};
    std::string command{"gst-launch-1.0"};
};

[[nodiscard]] std::vector<std::string> make_gstreamer_camera_arguments(
    const GStreamerCameraConfig& config);

[[nodiscard]] std::unique_ptr<mission::ports::CameraSource>
make_gstreamer_camera_source(GStreamerCameraConfig config = {});

} // namespace onboard_autonomy::hardware::camera
