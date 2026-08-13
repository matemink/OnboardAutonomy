#pragma once

#include "onboard_autonomy/adapters/mavlink/CommandAck.hpp"
#include "onboard_autonomy/adapters/mavlink/ParameterValue.hpp"
#include "onboard_autonomy/domain/VehicleState.hpp"

#include <ardupilotmega/mavlink.h>

#include <cstdint>
#include <functional>
#include <span>
#include <string_view>

namespace onboard_autonomy::adapters::mavlink {

struct MessageObservation {
    std::uint32_t message_id{0};
    std::uint8_t source_system{0};
    std::uint8_t source_component{0};
    std::string_view message_name;
};

class MavlinkDecoder {
  public:
    using CommandAckHandler =
        std::function<void(const CommandAck&, domain::TimePoint)>;
    using MessageHandler =
        std::function<void(const MessageObservation&, domain::TimePoint)>;
    using ParameterValueHandler =
        std::function<void(const ParameterValue&, domain::TimePoint)>;

    explicit MavlinkDecoder(domain::VehicleState& state,
        CommandAckHandler command_ack_handler = {},
        MessageHandler message_handler = {},
        ParameterValueHandler parameter_value_handler = {});

    void ingest(std::span<const std::uint8_t> bytes, domain::TimePoint now);

  private:
    void handle_message(const mavlink_message_t& message,
        domain::TimePoint now);
    void handle_heartbeat(const mavlink_message_t& message,
        domain::TimePoint now);
    void handle_gps(const mavlink_message_t& message, domain::TimePoint now);
    void handle_global_position(const mavlink_message_t& message,
        domain::TimePoint now);
    void handle_local_position(const mavlink_message_t& message,
        domain::TimePoint now);
    void handle_attitude(const mavlink_message_t& message,
        domain::TimePoint now);
    void handle_battery(const mavlink_message_t& message,
        domain::TimePoint now);
    void handle_system_status(const mavlink_message_t& message,
        domain::TimePoint now);
    void handle_autopilot_version(const mavlink_message_t& message);
    void handle_parameter_value(const mavlink_message_t& message,
        domain::TimePoint now);
    void handle_status_text(const mavlink_message_t& message,
        domain::TimePoint now);
    void handle_command_ack(const mavlink_message_t& message,
        domain::TimePoint now);

    domain::VehicleState& state_;
    CommandAckHandler command_ack_handler_;
    MessageHandler message_handler_;
    ParameterValueHandler parameter_value_handler_;
    mavlink_message_t receive_message_{};
    mavlink_status_t receive_status_{};
};

} // namespace onboard_autonomy::adapters::mavlink
