#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::mission::ports {

enum class CameraSourcePhase {
    starting,
    streaming,
    reconnecting,
    stopped,
    failed,
};

struct CameraFrame {
    std::uint64_t sequence{0};
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::vector<std::uint8_t> yuv420;
    std::optional<std::chrono::system_clock::time_point> captured_at;
    std::chrono::system_clock::time_point received_at;
};

struct CameraSourceStatus {
    CameraSourcePhase phase{CameraSourcePhase::starting};
    std::string description;
    std::string error;
    std::uint64_t produced_frames{0};
    std::uint64_t overwritten_frames{0};
    std::uint64_t restart_count{0};
};

class CameraSource {
  public:
    virtual ~CameraSource() = default;

    [[nodiscard]] virtual std::optional<CameraFrame> take_latest_frame() = 0;
    [[nodiscard]] virtual CameraSourceStatus status() const = 0;
};

} // namespace onboard_autonomy::mission::ports
