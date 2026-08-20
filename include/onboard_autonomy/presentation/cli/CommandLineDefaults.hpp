#pragma once

#include <cstdint>
#include <string_view>

namespace onboard_autonomy::presentation::cli::defaults {

inline constexpr std::string_view kUdpBindAddress{"0.0.0.0"};
inline constexpr std::uint16_t kMavlinkUdpPort{14550};
inline constexpr std::uint32_t kSerialBaudRate{115200};
inline constexpr std::uint32_t kCameraFramesPerSecond{30};
inline constexpr std::uint16_t kCameraUdpPort{5601};
inline constexpr std::uint16_t kCameraPreviewPort{8080};
inline constexpr std::uint32_t kCameraFrameWidth{640};
inline constexpr std::uint32_t kCameraFrameHeight{480};
inline constexpr std::uint32_t kSnapshotIntervalMs{1000};
inline constexpr double kDegreesPerCircle{360.0};
inline constexpr double kMillimetresPerMetre{1000.0};

} // namespace onboard_autonomy::presentation::cli::defaults
