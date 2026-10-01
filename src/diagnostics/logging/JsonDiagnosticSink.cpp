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
using detail::autonomy_phase_name;
using detail::link_event_json;
using detail::snapshot_json;
using detail::startup_phase_name;

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

  private:
    void write(const Json& record) {
        *output_ << record.dump() << '\n' << std::flush;
    }

    void event(const std::string_view name,
        const std::int64_t recorded_at_ms,
        const mission::AppSnapshot& snapshot,
        const std::string_view detail,
        Json context = Json::object()) {
        write({
            {"record_type", "event"},
            {"recorded_at_unix_ms", recorded_at_ms},
            {"elapsed_ms", snapshot.elapsed.count()},
            {"event", name},
            {"detail", detail},
            {"context", std::move(context)},
        });
    }

    void write_initial_event(const mission::AppSnapshot& snapshot,
        const std::int64_t recorded_at_ms) {
        event("runtime_observation_started",
            recorded_at_ms,
            snapshot,
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
                snapshot,
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
                snapshot,
                detail);
        }
    }

    void write_phase_events(const mission::AppSnapshot& previous,
        const mission::AppSnapshot& snapshot,
        const std::int64_t recorded_at_ms) {
        if (previous.flight_startup.phase != snapshot.flight_startup.phase) {
            event("flight_startup_phase_changed",
                recorded_at_ms,
                snapshot,
                snapshot.flight_startup.detail,
                {{"from", startup_phase_name(previous.flight_startup.phase)},
                    {"to", startup_phase_name(snapshot.flight_startup.phase)}});
        }
        if (previous.autonomy.phase != snapshot.autonomy.phase) {
            event("autonomy_phase_changed",
                recorded_at_ms,
                snapshot,
                snapshot.autonomy.detail,
                {{"from", autonomy_phase_name(previous.autonomy.phase)},
                    {"to", autonomy_phase_name(snapshot.autonomy.phase)}});
        }
        if (previous.companion_link_failsafe.phase !=
            snapshot.companion_link_failsafe.phase) {
            event("companion_link_failsafe_phase_changed",
                recorded_at_ms,
                snapshot,
                snapshot.companion_link_failsafe.detail,
                {{"from",
                     mission::companion_link_failsafe_phase_name(
                         previous.companion_link_failsafe.phase)},
                    {"to",
                        mission::companion_link_failsafe_phase_name(
                            snapshot.companion_link_failsafe.phase)}});
        }
        if (previous.motion_commands_allowed !=
            snapshot.motion_commands_allowed) {
            event("motion_safety_changed",
                recorded_at_ms,
                snapshot,
                snapshot.motion_commands_allowed ? "motion commands allowed"
                                                 : "motion commands blocked");
        }
    }

    void write_link_events(const mission::AppSnapshot& snapshot,
        const std::int64_t recorded_at_ms) {
        for (const auto& link_event : snapshot.link_events) {
            if (link_event.sequence <= last_link_event_sequence_) {
                continue;
            }
            event("mavlink_command_event",
                recorded_at_ms,
                snapshot,
                link_event.detail,
                link_event_json(link_event));
            last_link_event_sequence_ = link_event.sequence;
        }
    }

    void write_events(const mission::AppSnapshot& snapshot,
        const std::int64_t recorded_at_ms) {
        if (!previous_.has_value()) {
            write_initial_event(snapshot, recorded_at_ms);
        } else {
            const auto& previous = previous_.value();
            write_connection_events(previous, snapshot, recorded_at_ms);
            write_phase_events(previous, snapshot, recorded_at_ms);
        }
        write_link_events(snapshot, recorded_at_ms);
    }

    std::ofstream owned_output_;
    std::ostream* output_;
    std::optional<mission::AppSnapshot> previous_;
    std::uint64_t last_link_event_sequence_{0};
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

} // namespace onboard_autonomy::diagnostics::logging
