#include "onboard_autonomy/application/VisionMonitor.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

namespace onboard_autonomy::application {

class VisionMonitor::Impl {
  public:
    explicit Impl(ports::TargetDetector& detector,
        TargetTrackerConfig tracker_config)
        : detector_(detector), target_tracker_(tracker_config) {}

    const std::vector<domain::TargetObservation>&
    process(const ports::CameraFrame& frame, const domain::TimePoint now) {
        auto batch = detector_.detect(frame);
        ++processed_frames_;

        const double processing_ms =
            std::chrono::duration<double, std::milli>(batch.processing_time)
                .count();
        latest_processing_ms_ = processing_ms;
        total_processing_ms_ += processing_ms;
        maximum_processing_ms_ =
            std::max(maximum_processing_ms_, processing_ms);

        latest_targets_ = std::move(batch.targets);
        total_targets_ += latest_targets_.size();
        if (!latest_targets_.empty()) {
            ++frames_with_targets_;
            last_detection_at_ = now;
        }
        target_tracker_.update(latest_targets_, now);
        const auto track = target_tracker_.snapshot(now);
        if (track.target_id.has_value() && track.position.has_value()) {
            const auto tracked = std::find_if(latest_targets_.begin(),
                latest_targets_.end(),
                [&track](const auto& target) {
                    return target.id == *track.target_id &&
                           target.pose.has_value();
                });
            if (tracked != latest_targets_.end()) {
                auto pose = tracked->pose;
                if (pose.has_value()) {
                    pose->position = track.position.value();
                    tracked->pose = pose;
                }
            }
        }
        return latest_targets_;
    }

    [[nodiscard]] VisionSnapshot snapshot(const domain::TimePoint now) const {
        VisionSnapshot result{
            .detector = detector_.description(),
            .processed_frames = processed_frames_,
            .frames_with_targets = frames_with_targets_,
            .total_targets = total_targets_,
            .latest_processing_ms = latest_processing_ms_,
            .average_processing_ms = std::nullopt,
            .maximum_processing_ms = std::nullopt,
            .last_detection_age_ms = std::nullopt,
            .latest_targets = latest_targets_,
            .target_track = target_tracker_.snapshot(now),
        };
        if (processed_frames_ > 0U) {
            result.average_processing_ms =
                total_processing_ms_ / static_cast<double>(processed_frames_);
            result.maximum_processing_ms = maximum_processing_ms_;
        }
        if (last_detection_at_.has_value() && now >= *last_detection_at_) {
            result.last_detection_age_ms =
                std::chrono::duration<double, std::milli>(
                    now - *last_detection_at_)
                    .count();
        }
        return result;
    }

  private:
    ports::TargetDetector& detector_;
    TargetTracker target_tracker_;
    std::uint64_t processed_frames_{0};
    std::uint64_t frames_with_targets_{0};
    std::uint64_t total_targets_{0};
    std::optional<domain::TimePoint> last_detection_at_;
    std::optional<double> latest_processing_ms_;
    double total_processing_ms_{0.0};
    double maximum_processing_ms_{0.0};
    std::vector<domain::TargetObservation> latest_targets_;
};

VisionMonitor::VisionMonitor(ports::TargetDetector& detector,
    TargetTrackerConfig tracker_config)
    : impl_(std::make_unique<Impl>(detector, tracker_config)) {}

VisionMonitor::~VisionMonitor() = default;
VisionMonitor::VisionMonitor(VisionMonitor&&) noexcept = default;
VisionMonitor& VisionMonitor::operator=(VisionMonitor&&) noexcept = default;

const std::vector<domain::TargetObservation>& VisionMonitor::process(
    const ports::CameraFrame& frame,
    const domain::TimePoint now) {
    return impl_->process(frame, now);
}

VisionSnapshot VisionMonitor::snapshot(const domain::TimePoint now) const {
    return impl_->snapshot(now);
}

} // namespace onboard_autonomy::application
