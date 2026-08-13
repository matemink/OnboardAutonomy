#include "onboard_autonomy/adapters/mavlink/TelemetryStreamConfigurator.hpp"

#include "onboard_autonomy/adapters/mavlink/MavlinkEncoder.hpp"

#include <ardupilotmega/mavlink.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <string_view>

namespace onboard_autonomy::adapters::mavlink {
namespace {

struct StreamRequest {
    std::uint32_t message_id;
    std::uint32_t interval_microseconds;
    std::string_view name;
};
constexpr std::array<StreamRequest, 6> kStreamRequests{{
    {
        MAVLINK_MSG_ID_SYS_STATUS,
        1'000'000,
        "SYS_STATUS at 1 Hz",
    },
    {
        MAVLINK_MSG_ID_GPS_RAW_INT,
        500'000,
        "GPS_RAW_INT at 2 Hz",
    },
    {
        MAVLINK_MSG_ID_BATTERY_STATUS,
        1'000'000,
        "BATTERY_STATUS at 1 Hz",
    },
    {
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        200'000,
        "GLOBAL_POSITION_INT at 5 Hz",
    },
    {
        MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        100'000,
        "LOCAL_POSITION_NED at 10 Hz",
    },
    {
        MAVLINK_MSG_ID_ATTITUDE,
        100'000,
        "ATTITUDE at 10 Hz",
    },
}};

constexpr auto kAcknowledgementTimeout = std::chrono::seconds(2);
constexpr std::size_t kMaximumAttempts = 3;

} // namespace

std::optional<std::vector<std::uint8_t>> TelemetryStreamConfigurator::update(
    const bool connected,
    const std::optional<std::uint8_t> vehicle_system_id,
    const domain::TimePoint now) {
    if (!connected || !vehicle_system_id.has_value()) {
        reset();
        return std::nullopt;
    }

    if (!vehicle_system_id_.has_value() ||
        *vehicle_system_id_ != *vehicle_system_id) {
        begin(*vehicle_system_id);
    }

    if (phase_ != TelemetrySetupPhase::configuring) {
        return std::nullopt;
    }

    if (awaiting_ack_) {
        if (now < acknowledgement_deadline_) {
            return std::nullopt;
        }
        if (attempt_ >= kMaximumAttempts) {
            phase_ = TelemetrySetupPhase::failed;
            failure_result_.reset();
            return std::nullopt;
        }
        awaiting_ack_ = false;
    }

    const auto& request = kStreamRequests.at(current_request_);
    const auto confirmation = static_cast<std::uint8_t>(attempt_);
    ++attempt_;
    awaiting_ack_ = true;
    acknowledgement_deadline_ = now + kAcknowledgementTimeout;

    return encode_set_message_interval(vehicle_system_id_.value_or(0U),
        request.message_id,
        request.interval_microseconds,
        confirmation);
}

void TelemetryStreamConfigurator::on_command_ack(
    const CommandAck& acknowledgement,
    const domain::TimePoint now) {
    if (phase_ != TelemetrySetupPhase::configuring || !awaiting_ack_ ||
        acknowledgement.command != MAV_CMD_SET_MESSAGE_INTERVAL ||
        !vehicle_system_id_.has_value() ||
        acknowledgement.source_system != *vehicle_system_id_) {
        return;
    }

    if (acknowledgement.target_system != 0 &&
        acknowledgement.target_system != *vehicle_system_id_) {
        return;
    }
    if (acknowledgement.target_component != 0 &&
        acknowledgement.target_component != kCompanionComponentId) {
        return;
    }

    if (acknowledgement.result == MAV_RESULT_IN_PROGRESS) {
        acknowledgement_deadline_ = now + kAcknowledgementTimeout;
        return;
    }

    if (acknowledgement.result != MAV_RESULT_ACCEPTED) {
        phase_ = TelemetrySetupPhase::failed;
        failure_result_ = acknowledgement.result;
        awaiting_ack_ = false;
        return;
    }

    ++completed_requests_;
    ++current_request_;
    attempt_ = 0;
    awaiting_ack_ = false;
    failure_result_.reset();

    if (current_request_ == kStreamRequests.size()) {
        phase_ = TelemetrySetupPhase::active;
    }
}

TelemetrySetupSnapshot TelemetryStreamConfigurator::snapshot() const {
    TelemetrySetupSnapshot result;
    result.phase = phase_;
    result.completed_requests = completed_requests_;
    result.total_requests = kStreamRequests.size();
    result.attempt = attempt_;
    result.failure_result = failure_result_;
    if (current_request_ < kStreamRequests.size()) {
        result.current_stream =
            std::string(kStreamRequests.at(current_request_).name);
    }
    return result;
}

void TelemetryStreamConfigurator::reset() {
    phase_ = TelemetrySetupPhase::waiting_for_vehicle;
    vehicle_system_id_.reset();
    current_request_ = 0;
    completed_requests_ = 0;
    attempt_ = 0;
    awaiting_ack_ = false;
    acknowledgement_deadline_ = {};
    failure_result_.reset();
}

void TelemetryStreamConfigurator::begin(const std::uint8_t vehicle_system_id) {
    reset();
    phase_ = TelemetrySetupPhase::configuring;
    vehicle_system_id_ = vehicle_system_id;
}

} // namespace onboard_autonomy::adapters::mavlink
