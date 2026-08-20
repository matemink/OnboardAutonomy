#include "onboard_autonomy/operator/ui/screen/ConsoleSnapshotSink.hpp"

#include "onboard_autonomy/operator/ui/screen/ConsoleView.hpp"

#include <chrono>
#include <ostream>
#include <utility>

namespace onboard_autonomy::operator_interface::ui {

class ConsoleSnapshotSink::Impl {
  public:
    Impl(std::ostream& output,
        std::string transport_description,
        const BoardTypeResolver* board_type_resolver,
        const bool use_color)
        : output_(output),
          transport_description_(std::move(transport_description)),
          board_type_resolver_(board_type_resolver), use_color_(use_color) {
        output_ << "\x1b[2J\x1b[H\x1b[?25l" << std::flush;
    }

    ~Impl() { output_ << "\x1b[?25h\x1b[0m\n" << std::flush; }

    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;

    void consume(const mission::AppSnapshot& snapshot) {
        output_ << "\x1b[H"
                << render_console(snapshot,
                       transport_description_,
                       use_color_,
                       board_type_resolver_)
                << std::flush;
    }

  private:
    std::ostream& output_;
    std::string transport_description_;
    const BoardTypeResolver* board_type_resolver_;
    bool use_color_;
};

ConsoleSnapshotSink::ConsoleSnapshotSink(std::ostream& output,
    std::string transport_description,
    const BoardTypeResolver* board_type_resolver,
    const bool use_color)
    : impl_(std::make_unique<Impl>(output,
          std::move(transport_description),
          board_type_resolver,
          use_color)) {}

ConsoleSnapshotSink::~ConsoleSnapshotSink() = default;

void ConsoleSnapshotSink::consume(const mission::AppSnapshot& snapshot,
    const std::chrono::system_clock::time_point recorded_at) {
    static_cast<void>(recorded_at);
    impl_->consume(snapshot);
}

} // namespace onboard_autonomy::operator_interface::ui
