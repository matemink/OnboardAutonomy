#pragma once

#include "onboard_autonomy/mission/cv/extrinsics/TargetTransform.hpp"

#include <filesystem>
#include <istream>

namespace onboard_autonomy::mission::cv {

class CameraExtrinsicsLoader {
  public:
    [[nodiscard]] static mission::CameraExtrinsics from_file(
        const std::filesystem::path& path);

    [[nodiscard]] static mission::CameraExtrinsics from_stream(
        std::istream& input);
};

} // namespace onboard_autonomy::mission::cv
