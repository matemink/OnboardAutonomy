#pragma once

#include "onboard_autonomy/mission/AppSnapshot.hpp"
#include "onboard_autonomy/operator/ui/screen/BoardTypeResolver.hpp"

#include <string>
#include <string_view>

namespace onboard_autonomy::operator_interface::ui {

std::string render_console(const mission::AppSnapshot& snapshot,
    std::string_view transport_description,
    bool use_color = true,
    const BoardTypeResolver* board_type_resolver = nullptr);

} // namespace onboard_autonomy::operator_interface::ui
