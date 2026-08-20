#pragma once

#include <chrono>

namespace onboard_autonomy::application {
struct AppSnapshot;
}

namespace onboard_autonomy::application::ports {

// Receives immutable runtime observations without becoming a mission
// dependency.
class RuntimeSnapshotSink {
  public:
    virtual ~RuntimeSnapshotSink() = default;

    virtual void consume(const AppSnapshot& snapshot,
        std::chrono::system_clock::time_point recorded_at) = 0;
};

} // namespace onboard_autonomy::application::ports
