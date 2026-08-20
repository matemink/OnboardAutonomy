#pragma once

#include "onboard_autonomy/diagnostics/preview/CameraPreviewSink.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace onboard_autonomy::diagnostics::preview {

struct HttpCameraPreviewConfig {
    static constexpr std::uint16_t kDefaultPort = 8080;
    static constexpr std::uint32_t kDefaultMaximumFramesPerSecond = 10;

    std::string bind_address{"0.0.0.0"};
    std::uint16_t port{kDefaultPort};
    std::uint32_t maximum_frames_per_second{kDefaultMaximumFramesPerSecond};
    std::filesystem::path page_file;
};

[[nodiscard]] std::unique_ptr<diagnostics::preview::CameraPreviewSink>
make_http_camera_preview_server(HttpCameraPreviewConfig config);

} // namespace onboard_autonomy::diagnostics::preview
