#pragma once

#include <QString>

#include <gz/gui/Plugin.hh>
#include <gz/gui/qt.h>

class WindIndicator final : public gz::gui::Plugin {
    Q_OBJECT
    Q_PROPERTY(
        double speedMetersPerSecond
        READ SpeedMetersPerSecond
        CONSTANT
    )
    Q_PROPERTY(
        double directionFromDegrees
        READ DirectionFromDegrees
        CONSTANT
    )
    Q_PROPERTY(
        double turbulenceMetersPerSecond
        READ TurbulenceMetersPerSecond
        CONSTANT
    )
    Q_PROPERTY(QString directionLabel READ DirectionLabel CONSTANT)

public:
    WindIndicator();
    ~WindIndicator() override = default;

    void LoadConfig(const tinyxml2::XMLElement* plugin_element) override;

    [[nodiscard]] double SpeedMetersPerSecond() const;
    [[nodiscard]] double DirectionFromDegrees() const;
    [[nodiscard]] double TurbulenceMetersPerSecond() const;
    [[nodiscard]] QString DirectionLabel() const;

private:
    double speed_m_s_{0.0};
    double direction_from_deg_{0.0};
    double turbulence_m_s_{0.0};
};
