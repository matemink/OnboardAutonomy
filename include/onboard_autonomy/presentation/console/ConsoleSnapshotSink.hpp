#pragma once

#include "onboard_autonomy/application/ports/RuntimeSnapshotSink.hpp"

#include <iosfwd>
#include <memory>
#include <string>

namespace onboard_autonomy::presentation {
class BoardTypeResolver;
}

namespace onboard_autonomy::presentation::console {

class ConsoleSnapshotSink final
    : public application::ports::RuntimeSnapshotSink {
  public:
    ConsoleSnapshotSink(std::ostream& output,
        std::string transport_description,
        const BoardTypeResolver* board_type_resolver,
        bool use_color = true);
    ~ConsoleSnapshotSink() override;

    ConsoleSnapshotSink(const ConsoleSnapshotSink&) = delete;
    ConsoleSnapshotSink& operator=(const ConsoleSnapshotSink&) = delete;

    void consume(const application::AppSnapshot& snapshot,
        std::chrono::system_clock::time_point recorded_at) override;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard_autonomy::presentation::console
