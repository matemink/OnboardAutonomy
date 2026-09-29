#pragma once

#if defined(__linux__)
#include <cerrno>
#include <chrono>
#include <csignal>
#include <initializer_list>
#include <thread>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace onboard_autonomy::hardware::camera {

inline bool camera_child_exited(const pid_t child, int& status) {
    const auto result = ::waitpid(child, &status, WNOHANG);
    return result == child || (result == -1 && errno == ECHILD);
}

// Called only by the capture worker, which exclusively owns and reaps the PID.
inline int stop_camera_child(const pid_t child) {
    constexpr auto kSignalGracePeriod = std::chrono::milliseconds{250};
    constexpr auto kExitPollInterval = std::chrono::milliseconds{10};
    int status = 0;
    if (camera_child_exited(child, status)) {
        return status;
    }
    for (const int signal : {SIGINT, SIGTERM}) {
        ::kill(child, signal);
        const auto deadline = std::chrono::steady_clock::now() + kSignalGracePeriod;
        while (std::chrono::steady_clock::now() < deadline) {
            if (camera_child_exited(child, status)) {
                return status;
            }
            std::this_thread::sleep_for(kExitPollInterval);
        }
    }
    ::kill(child, SIGKILL);
    while (::waitpid(child, &status, 0) == -1 && errno == EINTR) {
    }
    return status;
}

} // namespace onboard_autonomy::hardware::camera
#endif
