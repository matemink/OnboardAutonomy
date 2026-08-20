#include "onboard_autonomy/operator/ui/input/ConsoleInput.hpp"

#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <memory>
#include <optional>

namespace onboard_autonomy::operator_interface::ui {

class ConsoleInput::Impl {
  public:
    explicit Impl(const bool enabled) {
        if (!enabled || isatty(STDIN_FILENO) != 1) {
            return;
        }

        if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
            return;
        }

        original_flags_ = fcntl(STDIN_FILENO, F_GETFL);
        if (original_flags_ < 0) {
            return;
        }

        auto raw = original_termios_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
            return;
        }

        if (fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) != 0) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
            return;
        }

        active_ = true;
    }

    ~Impl() {
        if (!active_) {
            return;
        }

        fcntl(STDIN_FILENO, F_SETFL, original_flags_);
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    }

    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;

    [[nodiscard]] bool active() const { return active_; }

    [[nodiscard]] std::optional<char> poll() const {
        if (!active_) {
            return std::nullopt;
        }

        char input{};
        const auto received = read(STDIN_FILENO, &input, 1);
        if (received == 1) {
            return input;
        }
        if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
            errno != EINTR) {
            return std::nullopt;
        }
        return std::nullopt;
    }

  private:
    termios original_termios_{};
    int original_flags_{-1};
    bool active_{false};
};

ConsoleInput::ConsoleInput(const bool enabled)
    : impl_(std::make_unique<Impl>(enabled)) {}

ConsoleInput::~ConsoleInput() = default;

bool ConsoleInput::active() const { return impl_->active(); }

std::optional<char> ConsoleInput::poll() { return impl_->poll(); }

} // namespace onboard_autonomy::operator_interface::ui
