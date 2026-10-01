#pragma once

#include "onboard_autonomy/mission/cv/detection/TargetObservation.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace onboard_autonomy::mission {

struct VisionSnapshot {
    std::string detector;
    std::uint64_t processed_frames{0};
    std::uint64_t frames_with_targets{0};
    std::uint64_t total_targets{0};
    std::optional<double> latest_processing_ms;
    std::optional<double> average_processing_ms;
    std::optional<double> maximum_processing_ms;
    std::optional<double> last_detection_age_ms;
    std::vector<mission::TargetObservation> latest_targets;
};

} // namespace onboard_autonomy::mission
