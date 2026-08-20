#pragma once

#include "onboard_autonomy/mission/cv/calibration/CameraCalibration.hpp"

#include <filesystem>
#include <istream>

namespace onboard_autonomy::mission::cv {

class CameraCalibrationLoader {
  public:
    [[nodiscard]] static mission::CameraCalibration from_file(
        const std::filesystem::path& path);

    [[nodiscard]] static mission::CameraCalibration from_stream(
        std::istream& input);
};

} // namespace onboard_autonomy::mission::cv
