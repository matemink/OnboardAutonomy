#pragma once

#include "onboard_autonomy/application/ports/CameraPreviewSink.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace onboard_autonomy::adapters::preview {

struct HttpCameraPreviewConfig {
    static constexpr std::uint16_t kDefaultPort = 8080;
    static constexpr std::uint32_t kDefaultMaximumFramesPerSecond = 10;

    std::string bind_address{"0.0.0.0"};
    std::uint16_t port{kDefaultPort};
    std::uint32_t maximum_frames_per_second{kDefaultMaximumFramesPerSecond};
    std::filesystem::path page_file;
};

[[nodiscard]] std::unique_ptr<application::ports::CameraPreviewSink>
make_http_camera_preview_server(HttpCameraPreviewConfig config);

} // namespace onboard_autonomy::adapters::preview
