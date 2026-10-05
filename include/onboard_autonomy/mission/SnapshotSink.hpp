#pragma once

#include <chrono>

namespace onboard_autonomy::mission {
struct AppSnapshot;
struct LinkEvent;
struct CompanionLinkFailsafeSnapshot;
enum class CompanionLinkFailsafePhase;
}

namespace onboard_autonomy::mission::ports {

// Receives immutable runtime observations without becoming a mission
// dependency.
class RuntimeSnapshotSink {
  public:
    virtual ~RuntimeSnapshotSink() = default;

    virtual void consume(const mission::AppSnapshot& snapshot,
        std::chrono::system_clock::time_point recorded_at) = 0;

    // Delivered synchronously when observed, independently of snapshot cadence.
    virtual void consume_link_event(const mission::LinkEvent&,
        std::chrono::system_clock::time_point) {}
    virtual void consume_failsafe_transition(mission::CompanionLinkFailsafePhase,
        const mission::CompanionLinkFailsafeSnapshot&,
        std::chrono::milliseconds,
        std::chrono::system_clock::time_point) {}
};

} // namespace onboard_autonomy::mission::ports
