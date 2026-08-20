#pragma once

#include "onboard_autonomy/mission/cv/CameraSource.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace onboard_autonomy::hardware::camera {

struct RpicamCameraConfig {
    static constexpr std::uint32_t kDefaultWidth = 640;
    static constexpr std::uint32_t kDefaultHeight = 480;
    static constexpr std::uint32_t kDefaultFramesPerSecond = 30;
    static constexpr std::uint32_t kDefaultFrameTimeoutMs = 2000;
    static constexpr std::uint32_t kDefaultRestartDelayMs = 500;

    std::uint32_t width{kDefaultWidth};
    std::uint32_t height{kDefaultHeight};
    std::uint32_t frames_per_second{kDefaultFramesPerSecond};
    std::uint32_t camera_index{0};
    std::uint32_t frame_timeout_ms{kDefaultFrameTimeoutMs};
    std::uint32_t restart_delay_ms{kDefaultRestartDelayMs};
    std::string lens_position{"default"};
    std::string command{"rpicam-vid"};
};

[[nodiscard]] std::optional<std::int64_t> parse_rpicam_frame_wall_clock_ns(
    std::string_view line);

[[nodiscard]] std::unique_ptr<mission::ports::CameraSource>
make_rpicam_camera_source(RpicamCameraConfig config = {});

} // namespace onboard_autonomy::hardware::camera
