#pragma once

#include <cstdint>
#include <string>

namespace onboard_autonomy::hardware::mavlink {

struct ParameterValue {
    std::uint8_t source_system{0};
    std::uint8_t source_component{0};
    std::string id;
    double value{0.0};
    std::uint8_t type{0};
};

} // namespace onboard_autonomy::hardware::mavlink
