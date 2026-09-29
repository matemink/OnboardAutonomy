#include "onboard_autonomy/hardware/camera/GStreamerCameraSource.hpp"
#include "onboard_autonomy/hardware/camera/RpicamCameraSource.hpp"

#include <cerrno>
#include <chrono>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <sys/wait.h>

namespace {
using namespace std::chrono_literals;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <typename Predicate>
void wait_for(Predicate ready, const std::string& message) {
    const auto deadline = std::chrono::steady_clock::now() + 4s;
    while (!ready() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(5ms);
    }
    require(ready(), message);
}

template <typename Factory>
void check_lifecycle(Factory make_source) {
    auto source = make_source(200U);
    wait_for([&source] { return source->status().produced_frames > 0; },
        "fixture must publish a frame before stalling");
    wait_for([&source] { return source->status().restart_count > 0; },
        "stalled child ignoring SIGINT and SIGTERM must be restarted");
    source.reset();

    source = make_source(5000U);
    wait_for([&source] { return source->status().produced_frames > 0; },
        "fixture must install its signal handlers before shutdown");
    const auto start = std::chrono::steady_clock::now();
    source.reset();
    require(std::chrono::steady_clock::now() - start < 2s,
        "shutdown must escalate without waiting for the frame timeout");

    for (int iteration = 0; iteration < 20; ++iteration) {
        source = make_source(5000U);
        source.reset();
    }
    int status = 0;
    require(::waitpid(-1, &status, WNOHANG) == -1 && errno == ECHILD,
        "all camera children must be reaped, including immediate shutdowns");
}
} // namespace

int main(const int argc, char* argv[]) {
    try {
        require(argc == 2, "expected camera fixture executable path");
        const std::string command{argv[1]};
        check_lifecycle([&command](const std::uint32_t timeout) {
            return onboard_autonomy::hardware::camera::make_gstreamer_camera_source({
                .width = 8, .height = 2, .frame_timeout_ms = timeout,
                .restart_delay_ms = 25, .command = command});
        });
        check_lifecycle([&command](const std::uint32_t timeout) {
            return onboard_autonomy::hardware::camera::make_rpicam_camera_source({
                .width = 8, .height = 2, .frame_timeout_ms = timeout,
                .restart_delay_ms = 25, .command = command});
        });
        std::cout << "Camera process lifecycle tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
