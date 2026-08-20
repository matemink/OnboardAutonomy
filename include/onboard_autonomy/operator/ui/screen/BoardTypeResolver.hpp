#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::operator_interface::ui {

struct BoardTypeMatch {
    std::string preferred_name;
    std::vector<std::string> aliases;
};

class BoardTypeResolver {
  public:
    virtual ~BoardTypeResolver() = default;

    [[nodiscard]] virtual std::optional<BoardTypeMatch> resolve(
        std::uint16_t board_type) const = 0;
};

} // namespace onboard_autonomy::operator_interface::ui
