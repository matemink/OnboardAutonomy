#include "onboard_autonomy/mission/flight/VehicleState.hpp"

#include <algorithm>
#include <cctype>
#include <iterator>
#include <string_view>

namespace onboard_autonomy::mission {
namespace {

constexpr double kMillimetresPerMetre = 1000.0;

constexpr auto kHeartbeatTimeout = std::chrono::seconds(3);
constexpr auto kGpsTimeout = std::chrono::seconds(5);
constexpr auto kGlobalPositionTimeout = std::chrono::seconds(2);
constexpr auto kLocalPositionTimeout = std::chrono::seconds(2);
constexpr auto kAttitudeTimeout = std::chrono::seconds(2);
constexpr auto kBatteryTimeout = std::chrono::seconds(10);
constexpr auto kSystemStatusTimeout = std::chrono::seconds(5);
constexpr auto kWarningTimeout = std::chrono::seconds(30);
constexpr auto kBlockingWarningTimeout = std::chrono::seconds(5);
constexpr std::int8_t kMinimumBatteryPercent = 20;
constexpr std::uint32_t kBatterySensorFlag = 1U << 25U;
constexpr std::uint8_t kMavModeFlagSafetyArmed = 128;
constexpr std::uint8_t kMavSeverityWarning = 4;

bool is_fresh(const std::optional<TimePoint>& observed_at,
    TimePoint now,
    Clock::duration timeout) {
    return observed_at.has_value() && now - *observed_at <= timeout;
}

bool contains_case_insensitive(std::string_view text,
    std::string_view expected) {
    std::string lower;
    lower.reserve(text.size());
    std::transform(text.begin(),
        text.end(),
        std::back_inserter(lower),
        [](unsigned char value) {
            return static_cast<char>(std::tolower(value));
        });
    return lower.find(expected) != std::string::npos;
}

bool contains_prearm(std::string_view text) {
    return contains_case_insensitive(text, "prearm:");
}

bool contains_battery(std::string_view text) {
    return contains_case_insensitive(text, "battery");
}

} // namespace

void VehicleState::on_heartbeat(const std::uint8_t system_id,
    const std::uint8_t component_id,
    const std::uint8_t vehicle_type,
    const std::uint8_t autopilot_type,
    const std::uint8_t base_mode,
    const std::uint32_t custom_mode,
    const std::uint8_t system_status,
    const TimePoint now) {
    std::scoped_lock lock(mutex_);
    const bool reconnecting =
        last_heartbeat_.has_value() &&
        !is_fresh(last_heartbeat_, now, kHeartbeatTimeout);
    const bool controller_changed =
        (system_id_.has_value() && *system_id_ != system_id) ||
        (component_id_.has_value() && *component_id_ != component_id);
    if (reconnecting || controller_changed) {
        autopilot_metadata_.reset();
    }
    last_heartbeat_ = now;
    system_id_ = system_id;
    component_id_ = component_id;
    vehicle_type_ = vehicle_type;
    autopilot_type_ = autopilot_type;
    system_status_ = system_status;
    flight_mode_ = custom_mode;
    armed_ = (base_mode & kMavModeFlagSafetyArmed) != 0;
}

void VehicleState::on_gps(const std::uint8_t fix_type,
    const std::uint8_t satellites_visible,
    const TimePoint now) {
    std::scoped_lock lock(mutex_);
    last_gps_ = now;
    gps_fix_type_ = fix_type;
    satellites_visible_ = satellites_visible;
}

void VehicleState::on_global_position(const std::int32_t relative_altitude_mm,
    const TimePoint now) {
    std::scoped_lock lock(mutex_);
    last_global_position_ = now;
    relative_altitude_m_ =
        static_cast<double>(relative_altitude_mm) / kMillimetresPerMetre;
}

void VehicleState::on_local_position(const float north_m,
    const float east_m,
    const float down_m,
    const TimePoint now) {
    std::scoped_lock lock(mutex_);
    last_local_position_ = now;
    local_north_m_ = static_cast<double>(north_m);
    local_east_m_ = static_cast<double>(east_m);
    local_down_m_ = static_cast<double>(down_m);
}

void VehicleState::on_attitude(const float roll_rad,
    const float pitch_rad,
    const float yaw_rad,
    const float yaw_rate_rad_per_second,
    const TimePoint now) {
    std::scoped_lock lock(mutex_);
    last_attitude_ = now;
    roll_rad_ = static_cast<double>(roll_rad);
    pitch_rad_ = static_cast<double>(pitch_rad);
    yaw_rad_ = static_cast<double>(yaw_rad);
    yaw_rate_rad_per_second_ = static_cast<double>(yaw_rate_rad_per_second);
}

void VehicleState::update_battery_locked(const std::optional<double> voltage_v,
    const std::optional<double> current_a,
    const std::optional<std::int8_t> remaining_pct,
    const TimePoint now) {
    last_battery_ = now;
    if (voltage_v.has_value()) {
        battery_voltage_v_ = voltage_v;
    }
    if (current_a.has_value()) {
        battery_current_a_ = current_a;
    }
    if (remaining_pct.has_value()) {
        battery_remaining_pct_ = remaining_pct;
    }
}

void VehicleState::on_battery(const std::optional<double> voltage_v,
    const std::optional<double> current_a,
    const std::optional<std::int8_t> remaining_pct,
    const TimePoint now) {
    std::scoped_lock lock(mutex_);
    update_battery_locked(voltage_v, current_a, remaining_pct, now);
}

void VehicleState::on_battery_arming_voltage(const double voltage_v,
    const TimePoint now) {
    static_cast<void>(now);
    std::scoped_lock lock(mutex_);
    battery_arming_voltage_v_ = std::max(0.0, voltage_v);
}

void VehicleState::on_autopilot_metadata(const AutopilotMetadata metadata) {
    std::scoped_lock lock(mutex_);
    autopilot_metadata_ = metadata;
}

void VehicleState::on_system_status(const std::uint32_t sensors_enabled,
    const std::uint32_t sensors_healthy,
    const std::optional<double> voltage_v,
    const std::optional<double> current_a,
    const std::optional<std::int8_t> remaining_pct,
    const TimePoint now) {
    std::scoped_lock lock(mutex_);
    sensors_enabled_ = sensors_enabled;
    sensors_healthy_ = sensors_healthy;
    last_system_status_ = now;

    if (voltage_v.has_value() || current_a.has_value() ||
        remaining_pct.has_value()) {
        update_battery_locked(voltage_v, current_a, remaining_pct, now);
    }
}

void VehicleState::on_status_text(const std::uint8_t severity,
    std::string text,
    const TimePoint now) {
    if (text.empty() ||
        (severity > kMavSeverityWarning && !contains_prearm(text))) {
        return;
    }

    std::scoped_lock lock(mutex_);
    const auto existing = std::find_if(warnings_.begin(),
        warnings_.end(),
        [&text](const WarningEntry& warning) { return warning.text == text; });

    if (existing != warnings_.end()) {
        existing->last_seen = now;
        return;
    }

    warnings_.push_back(WarningEntry{
        .text = std::move(text),
        .last_seen = now,
    });
}

VehicleSnapshot VehicleState::snapshot(const TimePoint now) {
    std::scoped_lock lock(mutex_);

    std::erase_if(warnings_, [now](const WarningEntry& warning) {
        return now - warning.last_seen > kWarningTimeout;
    });

    VehicleSnapshot result;
    result.connected = is_fresh(last_heartbeat_, now, kHeartbeatTimeout);
    result.system_id = system_id_;
    result.component_id = component_id_;
    result.vehicle_type = vehicle_type_;
    result.autopilot_type = autopilot_type_;
    result.system_status = system_status_;
    result.flight_mode = flight_mode_;
    result.armed = armed_;
    result.autopilot_metadata = autopilot_metadata_;

    result.warnings.reserve(warnings_.size());
    std::transform(warnings_.begin(),
        warnings_.end(),
        std::back_inserter(result.warnings),
        [](const WarningEntry& warning) { return warning.text; });

    if (!result.connected) {
        return result;
    }

    const bool gps_fresh = is_fresh(last_gps_, now, kGpsTimeout);
    if (gps_fresh) {
        result.gps_fix_type = gps_fix_type_;
        result.satellites_visible = satellites_visible_;
        result.gps_ready = gps_fix_type_.has_value() && *gps_fix_type_ >= 3;
        result.navigation_ready = result.gps_ready;
    }

    if (is_fresh(last_global_position_, now, kGlobalPositionTimeout)) {
        result.relative_altitude_m = relative_altitude_m_;
    }

    if (is_fresh(last_local_position_, now, kLocalPositionTimeout)) {
        result.local_north_m = local_north_m_;
        result.local_east_m = local_east_m_;
        result.local_down_m = local_down_m_;
    }

    if (is_fresh(last_attitude_, now, kAttitudeTimeout)) {
        result.roll_rad = roll_rad_;
        result.pitch_rad = pitch_rad_;
        result.yaw_rad = yaw_rad_;
        result.yaw_rate_rad_per_second = yaw_rate_rad_per_second_;
    }

    const bool system_status_fresh =
        is_fresh(last_system_status_, now, kSystemStatusTimeout);
    const bool battery_sensor_healthy =
        system_status_fresh && (sensors_enabled_ & kBatterySensorFlag) != 0 &&
        (sensors_healthy_ & kBatterySensorFlag) != 0;
    const bool has_battery_warning = std::any_of(warnings_.begin(),
        warnings_.end(),
        [](const WarningEntry& warning) {
            return contains_battery(warning.text);
        });

    const bool battery_fresh = is_fresh(last_battery_, now, kBatteryTimeout);
    if (battery_fresh) {
        result.battery_voltage_v = battery_voltage_v_;
        result.battery_current_a = battery_current_a_;
        result.battery_remaining_pct = battery_remaining_pct_;
        result.battery_arming_voltage_v = battery_arming_voltage_v_;
        const bool voltage_meets_arming_threshold =
            battery_voltage_v_.has_value() &&
            battery_arming_voltage_v_.has_value() &&
            (*battery_arming_voltage_v_ == 0.0 ||
                *battery_voltage_v_ >= *battery_arming_voltage_v_);
        result.battery_ready =
            voltage_meets_arming_threshold && battery_sensor_healthy &&
            !has_battery_warning &&
            (!battery_remaining_pct_.has_value() ||
                *battery_remaining_pct_ >= kMinimumBatteryPercent);
    }

    result.system_health_known = system_status_fresh;
    result.system_health_ok =
        system_status_fresh && (sensors_enabled_ & ~sensors_healthy_) == 0;

    const bool has_recent_warning = std::any_of(warnings_.begin(),
        warnings_.end(),
        [now](const WarningEntry& warning) {
            return now - warning.last_seen <= kBlockingWarningTimeout;
        });

    result.armable = result.navigation_ready && result.battery_ready &&
                     result.system_health_ok && !has_recent_warning;
    return result;
}

} // namespace onboard_autonomy::mission
