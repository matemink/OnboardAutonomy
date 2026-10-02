#pragma once

#include "onboard_autonomy/mission/AppSnapshot.hpp"

#include <nlohmann/json_fwd.hpp>

#include <cstdint>
#include <string_view>

namespace onboard_autonomy::diagnostics::logging::detail {

[[nodiscard]] nlohmann::json snapshot_json(const mission::AppSnapshot& snapshot,
    std::int64_t recorded_at_unix_ms);
[[nodiscard]] nlohmann::json link_event_json(const mission::LinkEvent& event);

} // namespace onboard_autonomy::diagnostics::logging::detail
