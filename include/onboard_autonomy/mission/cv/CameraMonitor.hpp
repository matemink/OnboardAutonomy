#pragma once

#include "onboard_autonomy/mission/cv/VisionMonitor.hpp"
#include "onboard_autonomy/mission/cv/CameraSource.hpp"
#include "onboard_autonomy/mission/flight/VehicleState.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::mission {

struct CameraSnapshot {
    ports::CameraSourcePhase phase{ports::CameraSourcePhase::starting};
    std::string source;
    std::string error;
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::uint64_t received_frames{0};
    std::uint64_t dropped_before_processing{0};
    std::uint64_t camera_restarts{0};
    std::uint64_t frames_with_capture_timestamp{0};
    std::optional<double> measured_fps;
    std::optional<double> latest_latency_ms;
    std::optional<double> average_latency_ms;
    std::optional<double> maximum_latency_ms;
    std::optional<double> latest_frame_age_ms;
};

struct ProcessedCameraFrame {
    ports::CameraFrame frame;
    std::vector<mission::TargetObservation> targets;
    TargetTrackSnapshot target_track;
};

class CameraMonitor {
  public:
    explicit CameraMonitor(ports::CameraSource& source,
        ports::TargetDetector* target_detector = nullptr);
    ~CameraMonitor();

    CameraMonitor(const CameraMonitor&) = delete;
    CameraMonitor& operator=(const CameraMonitor&) = delete;
    CameraMonitor(CameraMonitor&&) noexcept;
    CameraMonitor& operator=(CameraMonitor&&) noexcept;

    void poll(mission::TimePoint now);
    [[nodiscard]] CameraSnapshot snapshot(mission::TimePoint now) const;
    [[nodiscard]] std::optional<VisionSnapshot> vision_snapshot(
        mission::TimePoint now) const;
    [[nodiscard]] std::optional<ProcessedCameraFrame>
    take_latest_processed_frame();
    void disable_target_detection();

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard_autonomy::mission
