#pragma once

#include <chrono>

namespace onboard_autonomy::mission {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

} // namespace onboard_autonomy::mission
