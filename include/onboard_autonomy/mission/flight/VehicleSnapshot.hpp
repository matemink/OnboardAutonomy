#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::mission {

struct AutopilotMetadata {
    std::uint8_t firmware_major{0};
    std::uint8_t firmware_minor{0};
    std::uint8_t firmware_patch{0};
    std::uint8_t firmware_release_type{0};
    std::uint64_t capabilities{0};
    std::uint32_t board_version{0};
    std::uint16_t vendor_id{0};
    std::uint16_t product_id{0};
};

struct VehicleSnapshot {
    bool connected{false};
    bool gps_ready{false};
    bool navigation_ready{false};
    bool battery_ready{false};
    bool system_health_known{false};
    bool system_health_ok{false};
    bool armable{false};
    bool armed{false};
    std::optional<std::uint8_t> system_id;
    std::optional<std::uint8_t> component_id;
    std::optional<std::uint8_t> vehicle_type;
    std::optional<std::uint8_t> autopilot_type;
    std::optional<std::uint8_t> system_status;
    std::optional<std::uint32_t> flight_mode;
    std::optional<std::uint8_t> gps_fix_type;
    std::optional<std::uint8_t> satellites_visible;
    std::optional<double> relative_altitude_m;
    std::optional<double> local_north_m;
    std::optional<double> local_east_m;
    std::optional<double> local_down_m;
    std::optional<double> roll_rad;
    std::optional<double> pitch_rad;
    std::optional<double> yaw_rad;
    std::optional<double> yaw_rate_rad_per_second;
    std::optional<double> battery_voltage_v;
    std::optional<double> battery_current_a;
    std::optional<std::int8_t> battery_remaining_pct;
    std::optional<double> battery_arming_voltage_v;
    std::optional<AutopilotMetadata> autopilot_metadata;
    std::vector<std::string> warnings;
};

} // namespace onboard_autonomy::mission
