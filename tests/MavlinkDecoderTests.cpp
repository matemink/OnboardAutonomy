#include "TestCases.hpp"

#include "onboard_autonomy/mission/flight/VehicleState.hpp"
#include "onboard_autonomy/hardware/mavlink/MavlinkDecoder.hpp"
#include "onboard_autonomy/hardware/mavlink/MavlinkEncoder.hpp"

#include <ardupilotmega/mavlink.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::vector<std::uint8_t> serialize(const mavlink_message_t& message) {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto length = mavlink_msg_to_send_buffer(buffer.data(), &message);
    return {buffer.begin(), buffer.begin() + length};
}

void partial_heartbeat_is_reassembled() {
    onboard_autonomy::mission::VehicleState state;
    std::optional<onboard_autonomy::hardware::mavlink::MessageObservation>
        observed;
    onboard_autonomy::hardware::mavlink::MavlinkDecoder decoder{
        state,
        {},
        [&observed](
            const onboard_autonomy::hardware::mavlink::MessageObservation&
                message,
            const onboard_autonomy::mission::TimePoint) { observed = message; },
    };
    const onboard_autonomy::mission::TimePoint now{};

    mavlink_message_t heartbeat{};
    mavlink_msg_heartbeat_pack(1,
        1,
        &heartbeat,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,
        0,
        MAV_STATE_STANDBY);

    const auto bytes = serialize(heartbeat);
    const auto split = bytes.size() / 2;
    decoder.ingest(std::span<const std::uint8_t>{bytes.data(), split}, now);
    require(!state.snapshot(now).connected,
        "partial MAVLink frame must not be emitted");
    require(!observed.has_value(),
        "partial MAVLink frame must not trigger link activity");

    decoder.ingest(
        std::span<const std::uint8_t>{
            bytes.data() + split,
            bytes.size() - split,
        },
        now);
    require(state.snapshot(now).connected,
        "decoder must preserve partial frame state");
    require(observed.has_value() &&
                observed->message_id == MAVLINK_MSG_ID_HEARTBEAT &&
                observed->source_system == 1 &&
                observed->source_component == 1 &&
                observed->message_name == "HEARTBEAT",
        "a complete frame must expose its real MAVLink identity");
}

void multiple_frames_in_one_read_are_decoded() {
    onboard_autonomy::mission::VehicleState state;
    std::size_t observed_messages = 0;
    onboard_autonomy::hardware::mavlink::MavlinkDecoder decoder{
        state,
        {},
        [&observed_messages](
            const onboard_autonomy::hardware::mavlink::MessageObservation&,
            const onboard_autonomy::mission::TimePoint) {
            ++observed_messages;
        },
    };
    const onboard_autonomy::mission::TimePoint now{};

    mavlink_message_t heartbeat{};
    mavlink_msg_heartbeat_pack(1,
        MAV_COMP_ID_AUTOPILOT1,
        &heartbeat,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,
        0,
        MAV_STATE_STANDBY);
    mavlink_gps_raw_int_t gps{};
    gps.fix_type = 3;
    gps.satellites_visible = 12;
    mavlink_message_t gps_message{};
    mavlink_msg_gps_raw_int_encode(1,
        MAV_COMP_ID_AUTOPILOT1,
        &gps_message,
        &gps);

    auto bytes = serialize(heartbeat);
    const auto gps_bytes = serialize(gps_message);
    bytes.insert(bytes.end(), gps_bytes.begin(), gps_bytes.end());
    decoder.ingest(bytes, now);

    const auto snapshot = state.snapshot(now);
    require(snapshot.connected && snapshot.gps_ready,
        "one read may update state from several MAVLink frames");
    require(observed_messages == 2,
        "every complete frame in one read must be observed");
}

void inbound_burst_preserves_every_frame() {
    onboard_autonomy::mission::VehicleState state;
    std::size_t observed_messages = 0;
    onboard_autonomy::hardware::mavlink::MavlinkDecoder decoder{
        state,
        {},
        [&observed_messages](
            const onboard_autonomy::hardware::mavlink::MessageObservation&,
            const onboard_autonomy::mission::TimePoint) {
            ++observed_messages;
        },
    };
    std::vector<std::uint8_t> burst;

    constexpr std::size_t kBurstSize = 32;
    for (std::size_t index = 0; index < kBurstSize; ++index) {
        mavlink_message_t heartbeat{};
        mavlink_msg_heartbeat_pack(1,
            MAV_COMP_ID_AUTOPILOT1,
            &heartbeat,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_ARDUPILOTMEGA,
            0,
            static_cast<std::uint32_t>(index),
            MAV_STATE_STANDBY);
        const auto frame = serialize(heartbeat);
        burst.insert(burst.end(), frame.begin(), frame.end());
    }

    decoder.ingest(burst, onboard_autonomy::mission::TimePoint{});
    require(observed_messages == kBurstSize,
        "decoder must preserve every frame in an inbound burst");
}

void minimum_message_set_updates_vehicle_state() {
    onboard_autonomy::mission::VehicleState state;
    onboard_autonomy::hardware::mavlink::MavlinkDecoder decoder{state};
    const onboard_autonomy::mission::TimePoint now{};

    mavlink_message_t heartbeat{};
    mavlink_msg_heartbeat_pack(1,
        1,
        &heartbeat,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,
        4,
        MAV_STATE_STANDBY);

    mavlink_gps_raw_int_t gps{};
    gps.fix_type = 3;
    gps.satellites_visible = 14;
    mavlink_message_t gps_message{};
    mavlink_msg_gps_raw_int_encode(1, 1, &gps_message, &gps);

    mavlink_sys_status_t system_status{};
    constexpr std::uint32_t healthy_sensor_flags =
        1U | MAV_SYS_STATUS_SENSOR_BATTERY;
    system_status.onboard_control_sensors_enabled = healthy_sensor_flags;
    system_status.onboard_control_sensors_health = healthy_sensor_flags;
    system_status.voltage_battery = 15200;
    system_status.current_battery = 40;
    system_status.battery_remaining = 88;
    mavlink_message_t status_message{};
    mavlink_msg_sys_status_encode(1, 1, &status_message, &system_status);

    mavlink_param_value_t battery_threshold{};
    battery_threshold.param_value = 10.5F;
    battery_threshold.param_type = MAV_PARAM_TYPE_REAL32;
    constexpr char battery_parameter_name[] = "BATT_ARM_VOLT";
    std::memcpy(battery_threshold.param_id,
        battery_parameter_name,
        sizeof(battery_parameter_name));
    mavlink_message_t battery_threshold_message{};
    mavlink_msg_param_value_encode(1,
        MAV_COMP_ID_AUTOPILOT1,
        &battery_threshold_message,
        &battery_threshold);

    mavlink_global_position_int_t position{};
    position.relative_alt = 2500;
    mavlink_message_t position_message{};
    mavlink_msg_global_position_int_encode(1, 1, &position_message, &position);

    mavlink_local_position_ned_t local_position{};
    local_position.x = 12.5F;
    local_position.y = -3.0F;
    local_position.z = -2.5F;
    mavlink_message_t local_position_message{};
    mavlink_msg_local_position_ned_encode(1,
        1,
        &local_position_message,
        &local_position);

    mavlink_attitude_t attitude{};
    attitude.roll = 0.1F;
    attitude.pitch = -0.2F;
    attitude.yaw = 1.5F;
    mavlink_message_t attitude_message{};
    mavlink_msg_attitude_encode(1, 1, &attitude_message, &attitude);

    for (const auto* message : {
             &heartbeat,
             &gps_message,
             &status_message,
             &battery_threshold_message,
             &position_message,
             &local_position_message,
             &attitude_message,
         }) {
        const auto bytes = serialize(*message);
        decoder.ingest(bytes, now);
    }

    const auto snapshot = state.snapshot(now);
    require(snapshot.connected, "HEARTBEAT must update connection");
    require(snapshot.flight_mode == 4, "HEARTBEAT must update flight mode");
    require(snapshot.gps_ready, "GPS_RAW_INT must update GPS state");
    require(snapshot.navigation_ready,
        "GPS must currently provide the navigation estimate");
    require(snapshot.battery_remaining_pct == 88,
        "SYS_STATUS must update battery state");
    require(snapshot.battery_arming_voltage_v == 10.5,
        "PARAM_VALUE must update BATT_ARM_VOLT");
    require(snapshot.relative_altitude_m == 2.5,
        "GLOBAL_POSITION_INT must update relative altitude");
    require(snapshot.local_north_m == 12.5 && snapshot.local_east_m == -3.0 &&
                snapshot.local_down_m == -2.5,
        "LOCAL_POSITION_NED must update local route state");
    require(snapshot.roll_rad.has_value() && snapshot.pitch_rad.has_value() &&
                snapshot.yaw_rad.has_value(),
        "ATTITUDE must update the precision-target transform");
    require(snapshot.armable, "healthy minimum set should be armable");
}

void statustext_prearm_is_extracted() {
    onboard_autonomy::mission::VehicleState state;
    onboard_autonomy::hardware::mavlink::MavlinkDecoder decoder{state};
    const onboard_autonomy::mission::TimePoint now{};

    mavlink_statustext_t status_text{};
    status_text.severity = MAV_SEVERITY_INFO;
    constexpr char warning[] = "PreArm: GPS 1: Bad fix";
    std::memcpy(status_text.text, warning, sizeof(warning));

    mavlink_message_t message{};
    mavlink_msg_statustext_encode(1, 1, &message, &status_text);
    const auto bytes = serialize(message);
    decoder.ingest(bytes, now);

    const auto snapshot = state.snapshot(now);
    require(snapshot.warnings.size() == 1,
        "PreArm STATUSTEXT must become a warning");
}

void autopilot_version_is_unpacked_into_domain_metadata() {
    onboard_autonomy::mission::VehicleState state;
    onboard_autonomy::hardware::mavlink::MavlinkDecoder decoder{state};
    const onboard_autonomy::mission::TimePoint now{};

    mavlink_autopilot_version_t version{};
    version.flight_sw_version =
        (4U << 24U) | (6U << 16U) | (3U << 8U) | FIRMWARE_VERSION_TYPE_OFFICIAL;
    version.capabilities =
        MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
    version.board_version = (53U << 16U) | 2U;
    version.vendor_id = 0x1209;
    version.product_id = 0x5740;

    mavlink_message_t message{};
    mavlink_msg_autopilot_version_encode(1,
        MAV_COMP_ID_AUTOPILOT1,
        &message,
        &version);
    decoder.ingest(serialize(message), now);

    const auto snapshot = state.snapshot(now);
    require(snapshot.autopilot_metadata.has_value(),
        "AUTOPILOT_VERSION must reach the domain model");
    const auto& metadata = *snapshot.autopilot_metadata;
    require(metadata.firmware_major == 4 && metadata.firmware_minor == 6 &&
                metadata.firmware_patch == 3 &&
                metadata.firmware_release_type ==
                    FIRMWARE_VERSION_TYPE_OFFICIAL,
        "packed firmware version must be decoded by byte");
    require(metadata.board_version == version.board_version &&
                metadata.vendor_id == version.vendor_id &&
                metadata.product_id == version.product_id &&
                metadata.capabilities == version.capabilities,
        "hardware metadata and capabilities must be preserved");
}

void companion_heartbeat_does_not_replace_autopilot() {
    onboard_autonomy::mission::VehicleState state;
    onboard_autonomy::hardware::mavlink::MavlinkDecoder decoder{state};
    const onboard_autonomy::mission::TimePoint now{};

    mavlink_message_t autopilot{};
    mavlink_msg_heartbeat_pack(1,
        MAV_COMP_ID_AUTOPILOT1,
        &autopilot,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,
        0,
        MAV_STATE_STANDBY);
    decoder.ingest(serialize(autopilot), now);

    mavlink_message_t companion{};
    mavlink_msg_heartbeat_pack(1,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &companion,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0,
        0,
        MAV_STATE_ACTIVE);
    decoder.ingest(serialize(companion), now);

    const auto snapshot = state.snapshot(now);
    require(snapshot.component_id == MAV_COMP_ID_AUTOPILOT1,
        "companion heartbeat must not replace autopilot identity");
    require(snapshot.vehicle_type == MAV_TYPE_QUADROTOR,
        "companion heartbeat must not replace vehicle type");
}

void command_ack_is_forwarded_to_its_handler() {
    onboard_autonomy::mission::VehicleState state;
    std::optional<onboard_autonomy::hardware::mavlink::CommandAck> observed;
    onboard_autonomy::mission::TimePoint observed_at{};
    onboard_autonomy::hardware::mavlink::MavlinkDecoder decoder{
        state,
        [&observed,
            &observed_at](const onboard_autonomy::hardware::mavlink::CommandAck&
                              acknowledgement,
            const onboard_autonomy::mission::TimePoint now) {
            observed = acknowledgement;
            observed_at = now;
        },
    };
    const onboard_autonomy::mission::TimePoint now{std::chrono::seconds(5)};

    mavlink_message_t message{};
    mavlink_msg_command_ack_pack(1,
        MAV_COMP_ID_AUTOPILOT1,
        &message,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        MAV_RESULT_ACCEPTED,
        100,
        0,
        1,
        onboard_autonomy::hardware::mavlink::kCompanionComponentId);
    decoder.ingest(serialize(message), now);

    require(observed.has_value(), "COMMAND_ACK must reach its handler");
    require(observed->source_system == 1 &&
                observed->source_component == MAV_COMP_ID_AUTOPILOT1,
        "ACK source identity must be preserved");
    require(observed->command == MAV_CMD_SET_MESSAGE_INTERVAL &&
                observed->result == MAV_RESULT_ACCEPTED,
        "ACK command and result must be preserved");
    require(observed->target_component ==
                onboard_autonomy::hardware::mavlink::kCompanionComponentId,
        "ACK target component must be preserved");
    require(observed_at == now, "ACK receive time must be preserved");
}

void parameter_value_is_forwarded_to_its_handler() {
    onboard_autonomy::mission::VehicleState state;
    std::optional<onboard_autonomy::hardware::mavlink::ParameterValue> observed;
    onboard_autonomy::hardware::mavlink::MavlinkDecoder decoder{
        state,
        {},
        {},
        [&observed](const onboard_autonomy::hardware::mavlink::ParameterValue&
                        parameter,
            const onboard_autonomy::mission::TimePoint) {
            observed = parameter;
        },
    };

    mavlink_param_value_t parameter{};
    parameter.param_value = 5.0F;
    parameter.param_type = MAV_PARAM_TYPE_INT8;
    constexpr char parameter_name[] = "FS_GCS_ENABLE";
    std::memcpy(parameter.param_id, parameter_name, sizeof(parameter_name));
    mavlink_message_t message{};
    mavlink_msg_param_value_encode(1,
        MAV_COMP_ID_AUTOPILOT1,
        &message,
        &parameter);

    decoder.ingest(serialize(message), onboard_autonomy::mission::TimePoint{});
    require(observed.has_value() && observed->source_system == 1 &&
                observed->source_component == MAV_COMP_ID_AUTOPILOT1 &&
                observed->id == "FS_GCS_ENABLE" && observed->value == 5.0 &&
                observed->type == MAV_PARAM_TYPE_INT8,
        "PARAM_VALUE handler must preserve typed parameter data");
}

} // namespace

void run_mavlink_decoder_tests() {
    partial_heartbeat_is_reassembled();
    multiple_frames_in_one_read_are_decoded();
    inbound_burst_preserves_every_frame();
    minimum_message_set_updates_vehicle_state();
    statustext_prearm_is_extracted();
    autopilot_version_is_unpacked_into_domain_metadata();
    companion_heartbeat_does_not_replace_autopilot();
    command_ack_is_forwarded_to_its_handler();
    parameter_value_is_forwarded_to_its_handler();
}
