#include "onboard_autonomy/diagnostics/preview/HttpCameraPreviewServer.hpp"

#include <httplib.h>

#include <atomic>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace onboard_autonomy::diagnostics::preview {
namespace {

constexpr std::uint32_t kMaximumSupportedPreviewFps = 60;
constexpr auto kMicrosecondsPerSecond =
    std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::seconds{1})
        .count();
constexpr int kHttpNoContent = 204;
constexpr std::size_t kPreviewStreamCount = 2;
constexpr auto kMaximumFrameAge = std::chrono::seconds{2};

struct PreviewFrame {
    std::uint64_t sequence{0};
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::chrono::steady_clock::time_point published_at{};
    std::vector<std::uint8_t> luma;
    std::vector<mission::TargetObservation> targets;
    mission::TargetTrackSnapshot target_track;
};

std::size_t stream_index(const CameraPreviewStream stream) {
    switch (stream) {
    case CameraPreviewStream::downward:
        return 0U;
    case CameraPreviewStream::forward:
        return 1U;
    }
    throw std::invalid_argument("unknown camera preview stream");
}

std::string_view stream_name(const CameraPreviewStream stream) {
    switch (stream) {
    case CameraPreviewStream::downward:
        return "downward";
    case CameraPreviewStream::forward:
        return "forward";
    }
    return "unknown";
}

std::chrono::microseconds checked_frame_interval(
    const std::uint32_t maximum_frames_per_second) {
    if (maximum_frames_per_second == 0U ||
        maximum_frames_per_second > kMaximumSupportedPreviewFps) {
        throw std::invalid_argument(
            "preview frame rate must be between 1 and 60");
    }
    return std::chrono::microseconds{
        kMicrosecondsPerSecond / maximum_frames_per_second,
    };
}

std::string read_page(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error(
            "unable to open camera preview page: " + path.string());
    }
    return {
        std::istreambuf_iterator<char>{input},
        std::istreambuf_iterator<char>{},
    };
}

std::string targets_json(
    const std::vector<mission::TargetObservation>& targets) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(2) << '[';
    bool first_target = true;
    for (const auto& target : targets) {
        if (!first_target) {
            output << ',';
        }
        first_target = false;
        output << "{\"id\":" << target.id << ",\"center\":["
               << target.center.x_px << ',' << target.center.y_px << ']'
               << ",\"corners\":[";
        bool first_corner = true;
        for (const auto& corner : target.corners) {
            if (!first_corner) {
                output << ',';
            }
            first_corner = false;
            output << '[' << corner.x_px << ',' << corner.y_px << ']';
        }
        output << "],\"decision_margin\":" << target.decision_margin
               << ",\"corrected_bits\":" << target.corrected_bits
               << ",\"pose\":";
        const auto pose = target.pose;
        if (!pose.has_value()) {
            output << "null";
        } else {
            const auto& pose_value = pose.value();
            output << "{\"right_m\":" << pose_value.position.right_m
                   << ",\"down_m\":" << pose_value.position.down_m
                   << ",\"forward_m\":" << pose_value.position.forward_m
                   << ",\"object_space_error\":"
                   << pose_value.object_space_error << '}';
        }
        output << '}';
    }
    output << ']';
    return output.str();
}

std::string_view target_track_phase_name(
    const mission::TargetTrackPhase phase) {
    switch (phase) {
    case mission::TargetTrackPhase::searching:
        return "searching";
    case mission::TargetTrackPhase::acquiring:
        return "acquiring";
    case mission::TargetTrackPhase::tracking:
        return "tracking";
    }
    return "searching";
}

std::string target_track_json(const mission::TargetTrackSnapshot& track) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(3);
    output << "{\"phase\":\"" << target_track_phase_name(track.phase) << '"';
    output << ",\"target_id\":";
    if (track.target_id.has_value()) {
        output << *track.target_id;
    } else {
        output << "null";
    }
    output << ",\"consecutive_observations\":"
           << track.consecutive_observations;
    output << ",\"required_observations\":" << track.required_observations;
    output << ",\"observation_age_ms\":";
    if (track.observation_age_ms.has_value()) {
        output << *track.observation_age_ms;
    } else {
        output << "null";
    }
    output << ",\"position\":";
    if (track.position.has_value()) {
        output << "{\"right_m\":" << track.position->right_m
               << ",\"down_m\":" << track.position->down_m
               << ",\"forward_m\":" << track.position->forward_m << '}';
    } else {
        output << "null";
    }
    output << '}';
    return output.str();
}

class HttpCameraPreviewServer final
    : public diagnostics::preview::CameraPreviewSink {
  public:
    explicit HttpCameraPreviewServer(HttpCameraPreviewConfig config)
        : config_(std::move(config)), page_(read_page(config_.page_file)),
          minimum_frame_interval_(
              checked_frame_interval(config_.maximum_frames_per_second)) {
        if (config_.bind_address.empty() || config_.port == 0U) {
            throw std::invalid_argument(
                "invalid HTTP camera preview configuration");
        }

        server_.Get("/",
            [this](const httplib::Request&, httplib::Response& response) {
                response.set_header("Cache-Control", "no-store");
                response.set_content(page_, "text/html; charset=utf-8");
            });
        register_frame_route("/api/frame", CameraPreviewStream::downward);
        register_frame_route("/api/frame/downward",
            CameraPreviewStream::downward);
        register_frame_route("/api/frame/forward",
            CameraPreviewStream::forward);

        worker_ = std::jthread([this] {
            const bool listened = server_.listen(config_.bind_address,
                static_cast<int>(config_.port));
            if (!listened && !stopping_.load()) {
                failed_.store(true);
            }
        });
    }

    ~HttpCameraPreviewServer() override {
        stopping_.store(true);
        server_.stop();
    }

    HttpCameraPreviewServer(const HttpCameraPreviewServer&) = delete;
    HttpCameraPreviewServer& operator=(const HttpCameraPreviewServer&) = delete;

    void publish(const CameraPreviewStream stream,
        const mission::ports::CameraFrame& frame,
        const std::span<const mission::TargetObservation> targets,
        const mission::TargetTrackSnapshot& target_track) override {
        const auto now = std::chrono::steady_clock::now();
        std::scoped_lock lock(frame_mutex_);
        const auto index = stream_index(stream);
        const auto last_published_at =
            last_published_at_[index].value_or(now - minimum_frame_interval_);
        if (now - last_published_at < minimum_frame_interval_) {
            return;
        }

        const std::uint64_t luma_size =
            static_cast<std::uint64_t>(frame.width) *
            static_cast<std::uint64_t>(frame.height);
        if (frame.width == 0U || frame.height == 0U ||
            luma_size > frame.yuv420.size() ||
            luma_size > static_cast<std::uint64_t>(
                            std::numeric_limits<std::size_t>::max())) {
            return;
        }

        const auto end =
            frame.yuv420.begin() + static_cast<std::ptrdiff_t>(luma_size);
        latest_frames_[index] = PreviewFrame{
            .sequence = frame.sequence,
            .width = frame.width,
            .height = frame.height,
            .published_at = now,
            .luma = {frame.yuv420.begin(), end},
            .targets = {targets.begin(), targets.end()},
            .target_track = target_track,
        };
        last_published_at_[index] = now;
    }

    [[nodiscard]] std::string description() const override {
        return "http://" + config_.bind_address + ':' +
               std::to_string(config_.port) +
               (failed_.load() ? " (listen failed)" : "");
    }

  private:
    void register_frame_route(const std::string& path,
        const CameraPreviewStream stream) {
        server_.Get(path,
            [this, stream](const httplib::Request&,
                httplib::Response& response) {
                serve_frame(stream, response);
            });
    }

    void serve_frame(const CameraPreviewStream stream,
        httplib::Response& response) {
        PreviewFrame frame;
        {
            std::scoped_lock lock(frame_mutex_);
            const auto& latest = latest_frames_[stream_index(stream)];
            if (!latest.has_value()) {
                response.status = kHttpNoContent;
                response.set_header("Cache-Control", "no-store");
                return;
            }
            frame = latest.value();
            if (std::chrono::steady_clock::now() - frame.published_at >
                kMaximumFrameAge) {
                response.status = kHttpNoContent;
                response.set_header("Cache-Control", "no-store");
                return;
            }
        }

        response.set_header("Cache-Control", "no-store");
        response.set_header("X-Frame-Width", std::to_string(frame.width));
        response.set_header("X-Frame-Height", std::to_string(frame.height));
        response.set_header("X-Frame-Sequence", std::to_string(frame.sequence));
        response.set_header("X-OnboardAutonomy-Camera",
            std::string(stream_name(stream)));
        response.set_header("X-OnboardAutonomy-Targets",
            targets_json(frame.targets));
        response.set_header("X-OnboardAutonomy-Target-Track",
            target_track_json(frame.target_track));
        response.set_content(
            std::string{
                reinterpret_cast<const char*>(frame.luma.data()),
                frame.luma.size(),
            },
            "application/octet-stream");
    }

    HttpCameraPreviewConfig config_;
    std::string page_;
    std::chrono::microseconds minimum_frame_interval_;
    mutable std::mutex frame_mutex_;
    std::array<std::optional<PreviewFrame>, kPreviewStreamCount> latest_frames_;
    std::array<std::optional<std::chrono::steady_clock::time_point>,
        kPreviewStreamCount>
        last_published_at_;
    httplib::Server server_;
    std::atomic_bool stopping_{false};
    std::atomic_bool failed_{false};
    std::jthread worker_;
};

} // namespace

std::unique_ptr<diagnostics::preview::CameraPreviewSink>
make_http_camera_preview_server(HttpCameraPreviewConfig config) {
    return std::make_unique<HttpCameraPreviewServer>(std::move(config));
}

} // namespace onboard_autonomy::diagnostics::preview
