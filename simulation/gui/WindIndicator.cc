#include "WindIndicator.hh"

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <string_view>

#include <gz/plugin/Register.hh>

namespace {

std::optional<double> environment_number(const char* name) {
    const char* text = std::getenv(name);
    if (text == nullptr || *text == '\0') {
        return std::nullopt;
    }

    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0' ||
        !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

std::string_view cardinal_direction(const double degrees) {
    constexpr std::array<std::string_view, 8> directions{
        "N", "NE", "E", "SE", "S", "SW", "W", "NW"
    };
    const auto index = static_cast<std::size_t>(
        std::lround(degrees / 45.0)
    ) % directions.size();
    return directions[index];
}

}  // namespace

WindIndicator::WindIndicator()
    : gz::gui::Plugin(),
      speed_m_s_(
          environment_number("ONBOARD_AUTONOMY_WIND_SPEED_M_S")
              .value_or(0.0)
      ),
      direction_from_deg_(
          environment_number("ONBOARD_AUTONOMY_WIND_FROM_DEG")
              .value_or(0.0)
      ),
      turbulence_m_s_(
          environment_number(
              "ONBOARD_AUTONOMY_WIND_TURBULENCE_M_S"
          ).value_or(0.0)
      ) {}

void WindIndicator::LoadConfig(
    const tinyxml2::XMLElement* plugin_element
) {
    static_cast<void>(plugin_element);
}

double WindIndicator::SpeedMetersPerSecond() const {
    return speed_m_s_;
}

double WindIndicator::DirectionFromDegrees() const {
    return direction_from_deg_;
}

double WindIndicator::TurbulenceMetersPerSecond() const {
    return turbulence_m_s_;
}

QString WindIndicator::DirectionLabel() const {
    if (speed_m_s_ <= 0.0) {
        return QStringLiteral("CALM");
    }

    const double direction_to_deg =
        std::fmod(direction_from_deg_ + 180.0, 360.0);
    return QStringLiteral("%1 -> %2")
        .arg(QString::fromLatin1(
            cardinal_direction(direction_from_deg_).data()
        ))
        .arg(QString::fromLatin1(
            cardinal_direction(direction_to_deg).data()
        ));
}

GZ_ADD_PLUGIN(WindIndicator, gz::gui::Plugin)
