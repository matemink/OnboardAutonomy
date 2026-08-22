#include "onboard_autonomy/mission/cv/AsyncCameraMonitor.hpp"

#include <chrono>
#include <mutex>
#include <stop_token>
#include <thread>
#include <utility>

namespace onboard_autonomy::mission {
namespace {

constexpr auto kIdlePollDelay = std::chrono::milliseconds{2};

} // namespace

class AsyncCameraMonitor::Impl {
  public:
    explicit Impl(ports::CameraSource& source,
        ports::TargetDetector* target_detector)
        : monitor_(source, target_detector),
          worker_(
              [this](const std::stop_token& stop_token) { run(stop_token); }) {}

    ~Impl() {
        worker_.request_stop();
        worker_.join();
    }

    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    [[nodiscard]] std::optional<ProcessedCameraFrame>
    take_latest_processed_frame() {
        std::scoped_lock lock{latest_frame_mutex_};
        auto frame = std::move(latest_frame_);
        latest_frame_.reset();
        return frame;
    }

  private:
    void run(const std::stop_token& stop_token) {
        while (!stop_token.stop_requested()) {
            monitor_.poll(std::chrono::steady_clock::now());
            if (auto frame = monitor_.take_latest_processed_frame()) {
                std::scoped_lock lock{latest_frame_mutex_};
                latest_frame_ = std::move(frame);
            } else {
                std::this_thread::sleep_for(kIdlePollDelay);
            }
        }
    }

    CameraMonitor monitor_;
    std::mutex latest_frame_mutex_;
    std::optional<ProcessedCameraFrame> latest_frame_;
    std::jthread worker_;
};

AsyncCameraMonitor::AsyncCameraMonitor(ports::CameraSource& source,
    ports::TargetDetector* target_detector)
    : impl_(std::make_unique<Impl>(source, target_detector)) {}

AsyncCameraMonitor::~AsyncCameraMonitor() = default;

std::optional<ProcessedCameraFrame>
AsyncCameraMonitor::take_latest_processed_frame() {
    return impl_->take_latest_processed_frame();
}

} // namespace onboard_autonomy::mission
