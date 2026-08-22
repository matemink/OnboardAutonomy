#include "onboard_autonomy/mission/cv/AsyncCameraMonitor.hpp"

#include <chrono>
#include <exception>
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

    [[nodiscard]] std::optional<std::string> take_latest_error() {
        std::scoped_lock lock{latest_frame_mutex_};
        auto error = std::move(latest_error_);
        latest_error_.reset();
        return error;
    }

  private:
    void run(const std::stop_token& stop_token) {
        while (!stop_token.stop_requested()) {
            try {
                monitor_.poll(std::chrono::steady_clock::now());
            } catch (const std::exception& error) {
                disable_failed_detector(error.what());
            } catch (...) {
                disable_failed_detector("unknown target detector failure");
            }
            if (auto frame = monitor_.take_latest_processed_frame()) {
                std::scoped_lock lock{latest_frame_mutex_};
                latest_frame_ = std::move(frame);
            } else {
                std::this_thread::sleep_for(kIdlePollDelay);
            }
        }
    }

    void disable_failed_detector(std::string error) {
        monitor_.disable_target_detection();
        std::scoped_lock lock{latest_frame_mutex_};
        latest_error_ = std::move(error);
    }

    CameraMonitor monitor_;
    std::mutex latest_frame_mutex_;
    std::optional<ProcessedCameraFrame> latest_frame_;
    std::optional<std::string> latest_error_;
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

std::optional<std::string> AsyncCameraMonitor::take_latest_error() {
    return impl_->take_latest_error();
}

} // namespace onboard_autonomy::mission
