#pragma once

#include <memory>
#include <optional>

namespace onboard_autonomy::operator_interface::ui {

class ConsoleInput {
  public:
    explicit ConsoleInput(bool enabled);
    ~ConsoleInput();

    ConsoleInput(const ConsoleInput&) = delete;
    ConsoleInput& operator=(const ConsoleInput&) = delete;
    ConsoleInput(ConsoleInput&&) = delete;
    ConsoleInput& operator=(ConsoleInput&&) = delete;

    [[nodiscard]] bool active() const;
    [[nodiscard]] std::optional<char> poll();

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard_autonomy::operator_interface::ui
