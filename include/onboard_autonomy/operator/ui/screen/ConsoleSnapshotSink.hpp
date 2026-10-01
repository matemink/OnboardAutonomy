#pragma once

#include "onboard_autonomy/mission/SnapshotSink.hpp"
#include "onboard_autonomy/operator/ui/screen/ConsoleView.hpp"

#include <iosfwd>
#include <memory>
#include <string>

namespace onboard_autonomy::operator_interface::ui {
class BoardTypeResolver;
}

namespace onboard_autonomy::operator_interface::ui {

enum class ConsoleOutputMode { plain, terminal };

class ConsoleSnapshotSink final : public mission::ports::RuntimeSnapshotSink {
  public:
    ConsoleSnapshotSink(std::ostream& output,
        std::string transport_description,
        const BoardTypeResolver* board_type_resolver,
        ConsoleViewOptions options = {},
        ConsoleOutputMode output_mode = ConsoleOutputMode::plain);
    ~ConsoleSnapshotSink() override;

    ConsoleSnapshotSink(const ConsoleSnapshotSink&) = delete;
    ConsoleSnapshotSink& operator=(const ConsoleSnapshotSink&) = delete;

    void consume(const mission::AppSnapshot& snapshot,
        std::chrono::system_clock::time_point recorded_at) override;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard_autonomy::operator_interface::ui
