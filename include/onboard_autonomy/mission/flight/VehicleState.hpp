#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::mission {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

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
    std::optional<double> battery_voltage_v;
    std::optional<double> battery_current_a;
    std::optional<std::int8_t> battery_remaining_pct;
    std::optional<double> battery_arming_voltage_v;
    std::optional<AutopilotMetadata> autopilot_metadata;
    std::vector<std::string> warnings;
};

class VehicleState {
  public:
    void on_heartbeat(std::uint8_t system_id,
        std::uint8_t component_id,
        std::uint8_t vehicle_type,
        std::uint8_t autopilot_type,
        std::uint8_t base_mode,
        std::uint32_t custom_mode,
        std::uint8_t system_status,
        TimePoint now);

    void on_gps(std::uint8_t fix_type,
        std::uint8_t satellites_visible,
        TimePoint now);

    void on_global_position(std::int32_t relative_altitude_mm, TimePoint now);

    void
    on_local_position(float north_m, float east_m, float down_m, TimePoint now);

    void
    on_attitude(float roll_rad, float pitch_rad, float yaw_rad, TimePoint now);

    void on_battery(std::optional<double> voltage_v,
        std::optional<double> current_a,
        std::optional<std::int8_t> remaining_pct,
        TimePoint now);

    void on_battery_arming_voltage(double voltage_v, TimePoint now);

    void on_autopilot_metadata(AutopilotMetadata metadata);

    void on_system_status(std::uint32_t sensors_enabled,
        std::uint32_t sensors_healthy,
        std::optional<double> voltage_v,
        std::optional<double> current_a,
        std::optional<std::int8_t> remaining_pct,
        TimePoint now);

    void on_status_text(std::uint8_t severity, std::string text, TimePoint now);

    [[nodiscard]] VehicleSnapshot snapshot(TimePoint now);

  private:
    struct WarningEntry {
        std::string text;
        TimePoint last_seen;
    };

    void update_battery_locked(std::optional<double> voltage_v,
        std::optional<double> current_a,
        std::optional<std::int8_t> remaining_pct,
        TimePoint now);

    std::mutex mutex_;
    std::optional<TimePoint> last_heartbeat_;
    std::optional<TimePoint> last_gps_;
    std::optional<TimePoint> last_global_position_;
    std::optional<TimePoint> last_local_position_;
    std::optional<TimePoint> last_attitude_;
    std::optional<TimePoint> last_battery_;
    std::optional<TimePoint> last_system_status_;
    std::optional<std::uint8_t> system_id_;
    std::optional<std::uint8_t> component_id_;
    std::optional<std::uint8_t> vehicle_type_;
    std::optional<std::uint8_t> autopilot_type_;
    std::optional<std::uint8_t> system_status_;
    std::optional<std::uint32_t> flight_mode_;
    std::optional<std::uint8_t> gps_fix_type_;
    std::optional<std::uint8_t> satellites_visible_;
    std::optional<double> relative_altitude_m_;
    std::optional<double> local_north_m_;
    std::optional<double> local_east_m_;
    std::optional<double> local_down_m_;
    std::optional<double> roll_rad_;
    std::optional<double> pitch_rad_;
    std::optional<double> yaw_rad_;
    std::optional<double> battery_voltage_v_;
    std::optional<double> battery_current_a_;
    std::optional<std::int8_t> battery_remaining_pct_;
    std::optional<double> battery_arming_voltage_v_;
    std::optional<AutopilotMetadata> autopilot_metadata_;
    std::uint32_t sensors_enabled_{0};
    std::uint32_t sensors_healthy_{0};
    bool armed_{false};
    std::vector<WarningEntry> warnings_;
};

} // namespace onboard_autonomy::mission
