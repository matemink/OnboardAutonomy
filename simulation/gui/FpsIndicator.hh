#pragma once

#include <atomic>
#include <cstdint>

#include <QElapsedTimer>
#include <QTimer>

#include <gz/gui/Plugin.hh>
#include <gz/gui/qt.h>

class FpsIndicator final : public gz::gui::Plugin {
    Q_OBJECT
    Q_PROPERTY(double framesPerSecond READ FramesPerSecond NOTIFY
            FramesPerSecondChanged)

  public:
    FpsIndicator();
    ~FpsIndicator() override = default;

    void LoadConfig(const tinyxml2::XMLElement* plugin_element) override;

    [[nodiscard]] double FramesPerSecond() const;

  signals:
    void FramesPerSecondChanged();

  private:
    void UpdateFramesPerSecond();

    std::atomic<std::uint64_t> rendered_frame_count_{0U};
    QElapsedTimer sample_timer_{};
    QTimer update_timer_{};
    double frames_per_second_{0.0};
};
