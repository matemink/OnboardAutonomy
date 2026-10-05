#include "onboard_autonomy/bootstrap/CompanionRunner.hpp"

#include <csignal>
#include <stdexcept>

namespace onboard_autonomy::bootstrap {
namespace {

// std::signal requires process-lifetime state accessible to the handler.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
volatile std::sig_atomic_t keep_running = 1;
void handle_signal(int) {
    // Keep signal handling minimal; normal control flow owns all cleanup.
    keep_running = 0;
}

} // namespace

TerminationSignals::TerminationSignals()
    : previous_interrupt_(SIG_DFL), previous_terminate_(SIG_DFL) {
    keep_running = 1;
    previous_interrupt_ = std::signal(SIGINT, handle_signal);
    previous_terminate_ = std::signal(SIGTERM, handle_signal);
    if (previous_interrupt_ == SIG_ERR || previous_terminate_ == SIG_ERR) {
        if (previous_interrupt_ != SIG_ERR) {
            std::signal(SIGINT, previous_interrupt_);
        }
        if (previous_terminate_ != SIG_ERR) {
            std::signal(SIGTERM, previous_terminate_);
        }
        throw std::runtime_error("Unable to install termination handlers");
    }
}

TerminationSignals::~TerminationSignals() {
    std::signal(SIGINT, previous_interrupt_);
    std::signal(SIGTERM, previous_terminate_);
}

bool TerminationSignals::stop_requested() noexcept {
    return keep_running == 0;
}

void TerminationSignals::request_shutdown() noexcept {
    keep_running = 0;
}

} // namespace onboard_autonomy::bootstrap
