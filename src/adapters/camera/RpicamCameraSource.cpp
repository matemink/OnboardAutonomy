#include "onboard_autonomy/adapters/camera/RpicamCameraSource.hpp"

#include "../PosixError.hpp"

#include <atomic>
#include <charconv>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <cerrno>
#include <csignal>
#include <poll.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace onboard_autonomy::adapters::camera {
namespace {

constexpr std::string_view kFrameWallClockKey{"\"FrameWallClock\""};

std::size_t checked_frame_size(const RpicamCameraConfig& config) {
    if (config.width == 0U || config.height == 0U ||
        config.frames_per_second == 0U || config.frame_timeout_ms == 0U ||
        config.restart_delay_ms == 0U || config.width % 2U != 0U ||
        config.height % 2U != 0U) {
        throw std::invalid_argument(
            "camera width, height and FPS must be positive; "
            "recovery timings must be non-zero; YUV420 dimensions "
            "must be even");
    }

    const std::uint64_t pixels = static_cast<std::uint64_t>(config.width) *
                                 static_cast<std::uint64_t>(config.height);
    const std::uint64_t bytes = pixels + pixels / 2U;
    if (bytes >
        static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
        throw std::invalid_argument("camera frame size is too large");
    }
    return static_cast<std::size_t>(bytes);
}

#if defined(__linux__)

class RpicamCameraSource final : public application::ports::CameraSource {
  public:
    explicit RpicamCameraSource(RpicamCameraConfig config)
        : config_(std::move(config)), frame_size_(checked_frame_size(config_)),
          worker_(
              [this](const std::stop_token& stop_token) { run(stop_token); }) {
        std::scoped_lock lock(state_mutex_);
        status_.description =
            "rpicam-vid camera " + std::to_string(config_.camera_index);
    }

    ~RpicamCameraSource() override {
        worker_.request_stop();
        stop_child();
        metadata_ready_.notify_all();
    }

    [[nodiscard]] std::optional<application::ports::CameraFrame>
    take_latest_frame() override {
        std::scoped_lock lock(state_mutex_);
        auto frame = std::move(latest_frame_);
        latest_frame_.reset();
        return frame;
    }

    [[nodiscard]] application::ports::CameraSourceStatus
    status() const override {
        std::scoped_lock lock(state_mutex_);
        return status_;
    }

  private:
    enum class ReadResult {
        complete,
        stopped,
        end_of_file,
        timed_out,
        failed,
    };

    void run(const std::stop_token& stop_token) {
        while (!stop_token.stop_requested()) {
            prepare_attempt();
            run_once(stop_token);
            if (stop_token.stop_requested()) {
                break;
            }

            auto failure = status().error;
            if (failure.empty()) {
                failure = "rpicam camera stopped unexpectedly";
            }
            set_reconnecting(std::move(failure));
            if (!wait_for_restart(stop_token)) {
                break;
            }
        }
        set_stopped();
    }

    void run_once(const std::stop_token& stop_token) {
        int video_pipe[2]{-1, -1};
        int metadata_pipe[2]{-1, -1};
        int error_pipe[2]{-1, -1};
        if (::pipe(video_pipe) != 0 || ::pipe(metadata_pipe) != 0 ||
            ::pipe(error_pipe) != 0) {
            set_failure(
                "unable to create rpicam pipes: " + posix_error_message(errno));
            close_pipe(video_pipe);
            close_pipe(metadata_pipe);
            close_pipe(error_pipe);
            return;
        }

        const std::string metadata_path =
            "/proc/self/fd/" + std::to_string(metadata_pipe[1]);
        auto arguments = make_arguments(metadata_path);
        std::vector<char*> argv;
        argv.reserve(arguments.size() + 1U);
        for (auto& argument : arguments) {
            argv.push_back(argument.data());
        }
        argv.push_back(nullptr);

        const pid_t child = ::fork();
        if (child == -1) {
            set_failure(
                "unable to fork rpicam-vid: " + posix_error_message(errno));
            close_pipe(video_pipe);
            close_pipe(metadata_pipe);
            close_pipe(error_pipe);
            return;
        }

        if (child == 0) {
            ::close(video_pipe[0]);
            ::close(metadata_pipe[0]);
            ::close(error_pipe[0]);
            if (::dup2(video_pipe[1], STDOUT_FILENO) == -1 ||
                ::dup2(error_pipe[1], STDERR_FILENO) == -1) {
                _exit(126);
            }
            ::close(video_pipe[1]);
            ::close(error_pipe[1]);
            ::execvp(argv[0], argv.data());
            _exit(127);
        }

        child_pid_.store(child);
        ::close(video_pipe[1]);
        video_pipe[1] = -1;
        ::close(metadata_pipe[1]);
        metadata_pipe[1] = -1;
        ::close(error_pipe[1]);
        error_pipe[1] = -1;

        std::thread metadata_reader{[this, fd = metadata_pipe[0]] {
            read_lines(fd, [this](const std::string_view line) {
                const auto timestamp = parse_rpicam_frame_wall_clock_ns(line);
                if (!timestamp.has_value()) {
                    return;
                }
                {
                    std::scoped_lock lock(metadata_mutex_);
                    metadata_timestamps_.push_back(*timestamp);
                }
                metadata_ready_.notify_one();
            });
        }};
        std::thread error_reader{[this, fd = error_pipe[0]] {
            read_lines(fd, [this](const std::string_view line) {
                if (line.empty()) {
                    return;
                }
                std::scoped_lock lock(state_mutex_);
                last_process_message_ = std::string(line);
            });
        }};

        ReadResult read_result = ReadResult::complete;
        while (!stop_token.stop_requested()) {
            application::ports::CameraFrame frame{
                .sequence = 0U,
                .width = config_.width,
                .height = config_.height,
                .yuv420 = std::vector<std::uint8_t>(frame_size_),
                .captured_at = std::nullopt,
                .received_at = {},
            };
            read_result = read_exact(video_pipe[0],
                frame.yuv420,
                stop_token,
                std::chrono::milliseconds{config_.frame_timeout_ms});
            if (read_result != ReadResult::complete) {
                break;
            }
            frame.sequence = ++next_sequence_;
            frame.received_at = std::chrono::system_clock::now();

            const auto capture_timestamp = wait_for_metadata(stop_token);
            if (!capture_timestamp.has_value()) {
                if (!stop_token.stop_requested()) {
                    set_failure("camera frame arrived without FrameWallClock "
                                "metadata");
                }
                break;
            }
            frame.captured_at = std::chrono::system_clock::time_point{
                std::chrono::nanoseconds(*capture_timestamp),
            };
            publish(std::move(frame));
        }

        if (!stop_token.stop_requested()) {
            ::kill(child, SIGINT);
        }
        int child_status = 0;
        while (::waitpid(child, &child_status, 0) == -1 && errno == EINTR) {
        }
        child_pid_.store(-1);

        ::close(video_pipe[0]);
        metadata_reader.join();
        error_reader.join();
        ::close(metadata_pipe[0]);
        ::close(error_pipe[0]);

        if (stop_token.stop_requested()) {
            return;
        }
        if (status().phase == application::ports::CameraSourcePhase::failed) {
            return;
        }

        std::string detail;
        {
            std::scoped_lock lock(state_mutex_);
            detail = last_process_message_;
        }
        if (read_result == ReadResult::timed_out) {
            set_failure("rpicam frame stalled for " +
                        std::to_string(config_.frame_timeout_ms) + " ms");
        } else if (read_result == ReadResult::failed) {
            set_failure("failed to read the rpicam YUV stream");
        } else if (WIFEXITED(child_status)) {
            set_failure("rpicam-vid exited with status " +
                        std::to_string(WEXITSTATUS(child_status)) +
                        (detail.empty() ? "" : ": " + detail));
        } else if (WIFSIGNALED(child_status)) {
            set_failure("rpicam-vid stopped by signal " +
                        std::to_string(WTERMSIG(child_status)));
        } else {
            set_failure("rpicam YUV stream ended unexpectedly");
        }
    }

    [[nodiscard]] std::vector<std::string> make_arguments(
        const std::string& metadata_path) const {
        return {
            config_.command,
            "--camera",
            std::to_string(config_.camera_index),
            "--nopreview",
            "--timeout",
            "0",
            "--framerate",
            std::to_string(config_.frames_per_second),
            "--width",
            std::to_string(config_.width),
            "--height",
            std::to_string(config_.height),
            "--codec",
            "yuv420",
            "--flush",
            "--output",
            "-",
            "--metadata",
            metadata_path,
            "--metadata-format",
            "json",
            "--autofocus-mode",
            "manual",
            "--lens-position",
            config_.lens_position,
        };
    }

    static void close_pipe(int (&file_descriptors)[2]) {
        for (int& descriptor : file_descriptors) {
            if (descriptor >= 0) {
                ::close(descriptor);
                descriptor = -1;
            }
        }
    }

    template <typename Consumer>
    static void read_lines(const int fd, Consumer consume) {
        std::string pending;
        std::vector<char> buffer(4096);
        while (true) {
            const ssize_t count = ::read(fd, buffer.data(), buffer.size());
            if (count == 0) {
                break;
            }
            if (count < 0) {
                if (errno == EINTR) {
                    continue;
                }
                break;
            }
            pending.append(buffer.data(), static_cast<std::size_t>(count));
            std::size_t newline = 0;
            while ((newline = pending.find('\n')) != std::string::npos) {
                consume(std::string_view{pending}.substr(0, newline));
                pending.erase(0, newline + 1U);
            }
        }
        if (!pending.empty()) {
            consume(pending);
        }
    }

    static ReadResult read_exact(const int fd,
        std::vector<std::uint8_t>& destination,
        const std::stop_token& stop_token,
        const std::chrono::milliseconds frame_timeout) {
        std::size_t offset = 0;
        auto last_progress = std::chrono::steady_clock::now();
        while (offset < destination.size()) {
            if (stop_token.stop_requested()) {
                return ReadResult::stopped;
            }
            pollfd descriptor{
                .fd = fd,
                .events = POLLIN,
                .revents = 0,
            };
            const int ready = ::poll(&descriptor, 1, 100);
            if (ready == 0) {
                if (std::chrono::steady_clock::now() - last_progress >=
                    frame_timeout) {
                    return ReadResult::timed_out;
                }
                continue;
            }
            if (ready < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return ReadResult::failed;
            }
            const ssize_t count = ::read(fd,
                destination.data() + offset,
                destination.size() - offset);
            if (count == 0) {
                return ReadResult::end_of_file;
            }
            if (count < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return ReadResult::failed;
            }
            offset += static_cast<std::size_t>(count);
            last_progress = std::chrono::steady_clock::now();
        }
        return ReadResult::complete;
    }

    [[nodiscard]] std::optional<std::int64_t> wait_for_metadata(
        const std::stop_token& stop_token) {
        std::unique_lock lock(metadata_mutex_);
        const bool ready = metadata_ready_.wait_for(lock,
            std::chrono::milliseconds{config_.frame_timeout_ms},
            [this, stop_token] {
                return !metadata_timestamps_.empty() ||
                       stop_token.stop_requested();
            });
        if (!ready || metadata_timestamps_.empty()) {
            return std::nullopt;
        }
        const auto timestamp = metadata_timestamps_.front();
        metadata_timestamps_.pop_front();
        return timestamp;
    }

    void prepare_attempt() {
        {
            std::scoped_lock lock(state_mutex_);
            last_process_message_.clear();
            latest_frame_.reset();
        }
        {
            std::scoped_lock lock(metadata_mutex_);
            metadata_timestamps_.clear();
        }
    }

    [[nodiscard]] bool wait_for_restart(
        const std::stop_token& stop_token) const {
        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds{config_.restart_delay_ms};
        while (std::chrono::steady_clock::now() < deadline) {
            if (stop_token.stop_requested()) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{25});
        }
        return !stop_token.stop_requested();
    }

    void publish(application::ports::CameraFrame frame) {
        std::scoped_lock lock(state_mutex_);
        if (latest_frame_.has_value()) {
            ++status_.overwritten_frames;
        }
        latest_frame_ = std::move(frame);
        ++status_.produced_frames;
        status_.phase = application::ports::CameraSourcePhase::streaming;
        status_.error.clear();
    }

    void set_failure(std::string error) {
        std::scoped_lock lock(state_mutex_);
        status_.phase = application::ports::CameraSourcePhase::failed;
        status_.error = std::move(error);
    }

    void set_reconnecting(std::string error) {
        std::scoped_lock lock(state_mutex_);
        latest_frame_.reset();
        status_.phase = application::ports::CameraSourcePhase::reconnecting;
        status_.error = std::move(error);
        ++status_.restart_count;
    }

    void set_stopped() {
        std::scoped_lock lock(state_mutex_);
        status_.phase = application::ports::CameraSourcePhase::stopped;
    }

    void stop_child() {
        const pid_t child = child_pid_.load();
        if (child > 0) {
            ::kill(child, SIGINT);
        }
    }

    RpicamCameraConfig config_;
    std::size_t frame_size_{0};
    mutable std::mutex state_mutex_;
    application::ports::CameraSourceStatus status_;
    std::optional<application::ports::CameraFrame> latest_frame_;
    std::string last_process_message_;
    std::uint64_t next_sequence_{0U};
    std::mutex metadata_mutex_;
    std::condition_variable metadata_ready_;
    std::deque<std::int64_t> metadata_timestamps_;
    std::atomic<pid_t> child_pid_{-1};
    std::jthread worker_;
};

#endif

} // namespace

std::optional<std::int64_t> parse_rpicam_frame_wall_clock_ns(
    const std::string_view line) {
    const auto key = line.find(kFrameWallClockKey);
    if (key == std::string_view::npos) {
        return std::nullopt;
    }
    const auto colon = line.find(':', key + kFrameWallClockKey.size());
    if (colon == std::string_view::npos) {
        return std::nullopt;
    }

    const auto first = line.find_first_of("0123456789", colon + 1U);
    if (first == std::string_view::npos) {
        return std::nullopt;
    }
    const auto last = line.find_first_not_of("0123456789", first);
    const auto value = line.substr(first, last - first);

    std::int64_t timestamp = 0;
    const auto [end, error] =
        std::from_chars(value.data(), value.data() + value.size(), timestamp);
    if (error != std::errc{} || end != value.data() + value.size()) {
        return std::nullopt;
    }
    return timestamp;
}

std::unique_ptr<application::ports::CameraSource> make_rpicam_camera_source(
    RpicamCameraConfig config) {
    static_cast<void>(checked_frame_size(config));
#if defined(__linux__)
    return std::make_unique<RpicamCameraSource>(std::move(config));
#else
    throw std::runtime_error("rpicam camera source is only available on Linux");
#endif
}

} // namespace onboard_autonomy::adapters::camera
