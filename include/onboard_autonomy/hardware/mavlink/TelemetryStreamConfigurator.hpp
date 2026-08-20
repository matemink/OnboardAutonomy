#pragma once

#include "onboard_autonomy/mission/flight/VehicleState.hpp"
#include "onboard_autonomy/hardware/mavlink/CommandAck.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::hardware::mavlink {

enum class TelemetrySetupPhase {
    waiting_for_vehicle,
    configuring,
    active,
    failed,
};

struct TelemetrySetupSnapshot {
    TelemetrySetupPhase phase{TelemetrySetupPhase::waiting_for_vehicle};
    std::size_t completed_requests{0};
    std::size_t total_requests{0};
    std::string current_stream;
    std::size_t attempt{0};
    std::optional<std::uint8_t> failure_result;
};

class TelemetryStreamConfigurator {
  public:
    [[nodiscard]] std::optional<std::vector<std::uint8_t>> update(
        bool connected,
        std::optional<std::uint8_t> vehicle_system_id,
        mission::TimePoint now);

    void on_command_ack(const CommandAck& acknowledgement,
        mission::TimePoint now);

    [[nodiscard]] TelemetrySetupSnapshot snapshot() const;

  private:
    void reset();
    void begin(std::uint8_t vehicle_system_id);

    TelemetrySetupPhase phase_{TelemetrySetupPhase::waiting_for_vehicle};
    std::optional<std::uint8_t> vehicle_system_id_;
    std::size_t current_request_{0};
    std::size_t completed_requests_{0};
    std::size_t attempt_{0};
    bool awaiting_ack_{false};
    mission::TimePoint acknowledgement_deadline_{};
    std::optional<std::uint8_t> failure_result_;
};

} // namespace onboard_autonomy::hardware::mavlink
