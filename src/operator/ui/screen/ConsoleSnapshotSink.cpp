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
        ConsoleViewOptions options,
        const ConsoleOutputMode output_mode)
        : output_(output),
          transport_description_(std::move(transport_description)),
          board_type_resolver_(board_type_resolver), options_(options),
          output_mode_(output_mode) {
        if (output_mode_ == ConsoleOutputMode::terminal) {
            output_ << "\x1b[2J\x1b[H\x1b[?25l" << std::flush;
        } else {
            options_.use_color = false;
        }
    }

    ~Impl() {
        if (output_mode_ == ConsoleOutputMode::terminal) {
            output_ << "\x1b[?25h\x1b[0m\n" << std::flush;
        }
    }

    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;

    void consume(const mission::AppSnapshot& snapshot) {
        if (output_mode_ == ConsoleOutputMode::terminal) {
            output_ << "\x1b[H";
        }
        output_ << render_console(snapshot,
            transport_description_,
            options_,
            board_type_resolver_);
        output_ << (output_mode_ == ConsoleOutputMode::terminal ? "\x1b[J"
                                                                : "\n")
                << std::flush;
    }

  private:
    std::ostream& output_;
    std::string transport_description_;
    const BoardTypeResolver* board_type_resolver_;
    ConsoleViewOptions options_;
    ConsoleOutputMode output_mode_;
};

ConsoleSnapshotSink::ConsoleSnapshotSink(std::ostream& output,
    std::string transport_description,
    const BoardTypeResolver* board_type_resolver,
    ConsoleViewOptions options,
    const ConsoleOutputMode output_mode)
    : impl_(std::make_unique<Impl>(output,
          std::move(transport_description),
          board_type_resolver,
          options,
          output_mode)) {}

ConsoleSnapshotSink::~ConsoleSnapshotSink() = default;

void ConsoleSnapshotSink::consume(const mission::AppSnapshot& snapshot,
    const std::chrono::system_clock::time_point recorded_at) {
    static_cast<void>(recorded_at);
    impl_->consume(snapshot);
}

} // namespace onboard_autonomy::operator_interface::ui
