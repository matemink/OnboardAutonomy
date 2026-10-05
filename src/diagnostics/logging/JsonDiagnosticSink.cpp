#include "onboard_autonomy/diagnostics/logging/JsonDiagnosticSink.hpp"

#include "SnapshotJson.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace onboard_autonomy::diagnostics::logging {
namespace {

using Json = nlohmann::json;
using detail::link_event_json;
using detail::snapshot_json;

std::int64_t unix_milliseconds(
    const std::chrono::system_clock::time_point value) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        value.time_since_epoch())
        .count();
}

bool camera_streaming(const mission::AppSnapshot& snapshot) {
    return snapshot.camera.has_value() &&
           snapshot.camera->phase ==
               mission::ports::CameraSourcePhase::streaming;
}

} // namespace

class JsonDiagnosticSink::Impl {
  public:
    explicit Impl(std::ostream& output) : output_(&output) {}

    explicit Impl(const std::filesystem::path& output_file)
        : owned_output_(output_file, std::ios::app), output_(&owned_output_) {
        if (!owned_output_.is_open()) {
            throw std::runtime_error(
                "cannot open diagnostic log: " + output_file.string());
        }
    }

    void consume(const mission::AppSnapshot& snapshot,
        const std::chrono::system_clock::time_point recorded_at) {
        const auto recorded_at_ms = unix_milliseconds(recorded_at);
        write(snapshot_json(snapshot, recorded_at_ms));
        write_events(snapshot, recorded_at_ms);
        previous_ = snapshot;
    }

    void consume_link_event(const mission::LinkEvent& link_event,
        const std::chrono::system_clock::time_point recorded_at) {
        event("mavlink_command_event", unix_milliseconds(recorded_at),
            link_event.elapsed, link_event.detail, link_event_json(link_event));
    }

    void consume_failsafe_transition(const mission::CompanionLinkFailsafePhase previous,
        const mission::CompanionLinkFailsafeSnapshot& current,
        const std::chrono::milliseconds elapsed,
        const std::chrono::system_clock::time_point recorded_at) {
        event("companion_link_failsafe_phase_changed", unix_milliseconds(recorded_at),
            elapsed, current.detail,
            {{"from", mission::companion_link_failsafe_phase_name(previous)},
             {"to", mission::companion_link_failsafe_phase_name(current.phase)}});
    }

  private:
    void write(const Json& record) {
        *output_ << record.dump() << '\n' << std::flush;
    }

    void event(const std::string_view name,
        const std::int64_t recorded_at_ms,
        const std::chrono::milliseconds elapsed,
        const std::string_view detail,
        Json context = Json::object()) {
        write({
            {"record_type", "event"},
            {"recorded_at_unix_ms", recorded_at_ms},
            {"elapsed_ms", elapsed.count()},
            {"event", name},
            {"detail", detail},
            {"context", std::move(context)},
        });
    }

    void write_initial_event(const mission::AppSnapshot& snapshot,
        const std::int64_t recorded_at_ms) {
        event("runtime_observation_started",
            recorded_at_ms,
            snapshot.elapsed,
            "diagnostic sink attached",
            {{"vehicle_connected", snapshot.vehicle.connected},
                {"camera_streaming", camera_streaming(snapshot)}});
    }

    void write_connection_events(const mission::AppSnapshot& previous,
        const mission::AppSnapshot& snapshot,
        const std::int64_t recorded_at_ms) {
        if (previous.vehicle.connected != snapshot.vehicle.connected) {
            event(snapshot.vehicle.connected ? "flight_controller_recovered"
                                             : "flight_controller_lost",
                recorded_at_ms,
                snapshot.elapsed,
                snapshot.vehicle.connected ? "heartbeat recovered"
                                           : "heartbeat timed out");
        }

        const bool was_streaming = camera_streaming(previous);
        const bool is_streaming = camera_streaming(snapshot);
        if (was_streaming != is_streaming) {
            const std::string detail = snapshot.camera.has_value()
                                           ? snapshot.camera->error
                                           : "camera not configured";
            event(is_streaming ? "camera_stream_recovered"
                               : "camera_stream_stalled",
                recorded_at_ms,
                snapshot.elapsed,
                detail);
        }
    }

    void write_events(const mission::AppSnapshot& snapshot,
        const std::int64_t recorded_at_ms) {
        if (!previous_.has_value()) {
            write_initial_event(snapshot, recorded_at_ms);
        } else {
            const auto& previous = previous_.value();
            write_connection_events(previous, snapshot, recorded_at_ms);
        }
    }

    std::ofstream owned_output_;
    std::ostream* output_;
    std::optional<mission::AppSnapshot> previous_;
};

JsonDiagnosticSink::JsonDiagnosticSink(std::ostream& output)
    : impl_(std::make_unique<Impl>(output)) {}

JsonDiagnosticSink::JsonDiagnosticSink(const std::filesystem::path& output_file)
    : impl_(std::make_unique<Impl>(output_file)) {}

JsonDiagnosticSink::~JsonDiagnosticSink() = default;

void JsonDiagnosticSink::consume(const mission::AppSnapshot& snapshot,
    const std::chrono::system_clock::time_point recorded_at) {
    impl_->consume(snapshot, recorded_at);
}

void JsonDiagnosticSink::consume_link_event(const mission::LinkEvent& event,
    const std::chrono::system_clock::time_point recorded_at) {
    impl_->consume_link_event(event, recorded_at);
}

void JsonDiagnosticSink::consume_failsafe_transition(
    const mission::CompanionLinkFailsafePhase previous,
    const mission::CompanionLinkFailsafeSnapshot& current,
    const std::chrono::milliseconds elapsed,
    const std::chrono::system_clock::time_point recorded_at) {
    impl_->consume_failsafe_transition(previous, current, elapsed, recorded_at);
}

} // namespace onboard_autonomy::diagnostics::logging
