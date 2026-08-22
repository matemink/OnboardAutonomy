#include "onboard_autonomy/mission/cv/CameraMonitor.hpp"

#include <algorithm>
#include <chrono>
#include <span>
#include <utility>

namespace onboard_autonomy::mission {

class CameraMonitor::Impl {
  public:
    explicit Impl(ports::CameraSource& source,
        ports::TargetDetector* target_detector)
        : source_(source) {
        if (target_detector != nullptr) {
            vision_monitor_.emplace(*target_detector);
        }
    }

    void poll(const mission::TimePoint now) {
        auto frame = source_.take_latest_frame();
        if (!frame.has_value()) {
            return;
        }

        if (last_sequence_.has_value() &&
            frame->sequence > *last_sequence_ + 1U) {
            dropped_before_processing_ +=
                frame->sequence - *last_sequence_ - 1U;
        }
        last_sequence_ = frame->sequence;
        ++received_frames_;
        width_ = frame->width;
        height_ = frame->height;
        last_frame_observed_at_ = now;
        std::span<const mission::TargetObservation> targets;
        TargetTrackSnapshot target_track;
        if (vision_monitor_.has_value()) {
            targets = vision_monitor_->process(*frame, now);
            target_track = vision_monitor_->snapshot(now).target_track;
        }
        if (frame->captured_at.has_value()) {
            ++frames_with_capture_timestamp_;
            if (!first_capture_at_.has_value()) {
                first_capture_at_ = frame->captured_at;
            }
            last_capture_at_ = frame->captured_at;

            const auto latency = frame->received_at - *frame->captured_at;
            const double latency_ms =
                std::chrono::duration<double, std::milli>(latency).count();
            if (latency_ms >= 0.0) {
                ++valid_latency_samples_;
                latest_latency_ms_ = latency_ms;
                total_latency_ms_ += latency_ms;
                maximum_latency_ms_ = std::max(maximum_latency_ms_, latency_ms);
            }
        }

        latest_processed_frame_ = ProcessedCameraFrame{
            .frame = std::move(*frame),
            .targets = {targets.begin(), targets.end()},
            .target_track = target_track,
        };
    }

    [[nodiscard]] CameraSnapshot snapshot(const mission::TimePoint now) const {
        const auto source_status = source_.status();
        CameraSnapshot result{
            .phase = source_status.phase,
            .source = source_status.description,
            .error = source_status.error,
            .width = width_,
            .height = height_,
            .received_frames = received_frames_,
            .dropped_before_processing = std::max(dropped_before_processing_,
                source_status.overwritten_frames),
            .camera_restarts = source_status.restart_count,
            .frames_with_capture_timestamp = frames_with_capture_timestamp_,
            .measured_fps = std::nullopt,
            .latest_latency_ms = std::nullopt,
            .average_latency_ms = std::nullopt,
            .maximum_latency_ms = std::nullopt,
            .latest_frame_age_ms = std::nullopt,
        };

        if (first_capture_at_.has_value() && last_capture_at_.has_value() &&
            frames_with_capture_timestamp_ > 1U) {
            const double elapsed_seconds = std::chrono::duration<double>(
                *last_capture_at_ - *first_capture_at_)
                                               .count();
            if (elapsed_seconds > 0.0) {
                result.measured_fps =
                    static_cast<double>(frames_with_capture_timestamp_ - 1U) /
                    elapsed_seconds;
            }
        }

        result.latest_latency_ms = latest_latency_ms_;
        if (valid_latency_samples_ > 0U) {
            result.average_latency_ms =
                total_latency_ms_ / static_cast<double>(valid_latency_samples_);
            result.maximum_latency_ms = maximum_latency_ms_;
        }
        if (last_frame_observed_at_.has_value() &&
            now >= *last_frame_observed_at_) {
            result.latest_frame_age_ms =
                std::chrono::duration<double, std::milli>(
                    now - *last_frame_observed_at_)
                    .count();
        }
        return result;
    }

    [[nodiscard]] std::optional<VisionSnapshot> vision_snapshot(
        const mission::TimePoint now) const {
        if (!vision_monitor_.has_value()) {
            return std::nullopt;
        }
        return vision_monitor_->snapshot(now);
    }

    [[nodiscard]] std::optional<ProcessedCameraFrame>
    take_latest_processed_frame() {
        auto frame = std::move(latest_processed_frame_);
        latest_processed_frame_.reset();
        return frame;
    }

    void disable_target_detection() { vision_monitor_.reset(); }

  private:
    ports::CameraSource& source_;
    std::optional<VisionMonitor> vision_monitor_;
    std::optional<ProcessedCameraFrame> latest_processed_frame_;
    std::uint64_t received_frames_{0};
    std::uint64_t dropped_before_processing_{0};
    std::uint64_t frames_with_capture_timestamp_{0};
    std::uint64_t valid_latency_samples_{0};
    std::uint32_t width_{0};
    std::uint32_t height_{0};
    std::optional<std::uint64_t> last_sequence_;
    std::optional<mission::TimePoint> last_frame_observed_at_;
    std::optional<std::chrono::system_clock::time_point> first_capture_at_;
    std::optional<std::chrono::system_clock::time_point> last_capture_at_;
    std::optional<double> latest_latency_ms_;
    double total_latency_ms_{0.0};
    double maximum_latency_ms_{0.0};
};

CameraMonitor::CameraMonitor(ports::CameraSource& source,
    ports::TargetDetector* target_detector)
    : impl_(std::make_unique<Impl>(source, target_detector)) {}

CameraMonitor::~CameraMonitor() = default;
CameraMonitor::CameraMonitor(CameraMonitor&&) noexcept = default;
CameraMonitor& CameraMonitor::operator=(CameraMonitor&&) noexcept = default;

void CameraMonitor::poll(const mission::TimePoint now) { impl_->poll(now); }

CameraSnapshot CameraMonitor::snapshot(const mission::TimePoint now) const {
    return impl_->snapshot(now);
}

std::optional<VisionSnapshot> CameraMonitor::vision_snapshot(
    const mission::TimePoint now) const {
    return impl_->vision_snapshot(now);
}

std::optional<ProcessedCameraFrame>
CameraMonitor::take_latest_processed_frame() {
    return impl_->take_latest_processed_frame();
}

void CameraMonitor::disable_target_detection() {
    impl_->disable_target_detection();
}

} // namespace onboard_autonomy::mission
