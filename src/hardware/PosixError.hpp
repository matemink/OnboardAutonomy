#pragma once

#include <string>
#include <system_error>

namespace onboard_autonomy::hardware {

inline std::string posix_error_message(const int error_number) {
    return std::error_code(error_number, std::generic_category()).message();
}

} // namespace onboard_autonomy::hardware
