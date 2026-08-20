#pragma once

#include <cstdint>

namespace onboard_autonomy::hardware::mavlink {

struct CommandAck {
    std::uint8_t source_system{0};
    std::uint8_t source_component{0};
    std::uint16_t command{0};
    std::uint8_t result{0};
    std::uint8_t progress{0};
    std::int32_t result_parameter{0};
    std::uint8_t target_system{0};
    std::uint8_t target_component{0};
};

} // namespace onboard_autonomy::hardware::mavlink
