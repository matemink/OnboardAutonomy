#include "FpsIndicator.hh"

#include <QQuickWindow>

#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>

namespace {

constexpr int kSampleIntervalMilliseconds{500};
constexpr double kMillisecondsPerSecond{1000.0};

} // namespace

FpsIndicator::FpsIndicator() : gz::gui::Plugin() {}

void FpsIndicator::LoadConfig(const tinyxml2::XMLElement* plugin_element) {
    static_cast<void>(plugin_element);

    auto* main_window = gz::gui::App()->findChild<gz::gui::MainWindow*>();
    if (main_window == nullptr || main_window->QuickWindow() == nullptr) {
        return;
    }

    auto* quick_window = main_window->QuickWindow();
    connect(
        quick_window,
        &QQuickWindow::frameSwapped,
        this,
        [this]() {
            rendered_frame_count_.fetch_add(1U, std::memory_order_relaxed);
        },
        Qt::DirectConnection);

    update_timer_.setInterval(kSampleIntervalMilliseconds);
    connect(&update_timer_,
        &QTimer::timeout,
        this,
        &FpsIndicator::UpdateFramesPerSecond);

    sample_timer_.start();
    update_timer_.start();
}

double FpsIndicator::FramesPerSecond() const { return frames_per_second_; }

void FpsIndicator::UpdateFramesPerSecond() {
    const auto elapsed_milliseconds = sample_timer_.restart();
    const auto rendered_frames =
        rendered_frame_count_.exchange(0U, std::memory_order_relaxed);

    if (elapsed_milliseconds <= 0) {
        return;
    }

    frames_per_second_ = static_cast<double>(rendered_frames) *
                         kMillisecondsPerSecond /
                         static_cast<double>(elapsed_milliseconds);
    emit FramesPerSecondChanged();
}

GZ_ADD_PLUGIN(FpsIndicator, gz::gui::Plugin)
