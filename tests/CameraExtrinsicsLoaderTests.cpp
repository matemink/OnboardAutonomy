#include "TestCases.hpp"

#include "onboard_autonomy/mission/cv/extrinsics/CameraExtrinsicsLoader.hpp"

#include <cmath>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void gazebo_mount_loads_body_frd_transform() {
    const auto path = std::filesystem::path{ONBOARD_AUTONOMY_SOURCE_DIR} /
                      "config" / "gazebo-landing-camera-extrinsics.json";
    const auto extrinsics =
        onboard_autonomy::mission::cv::CameraExtrinsicsLoader::from_file(path);

    require(extrinsics.rotation_camera_to_body[1] == -1.0 &&
                extrinsics.rotation_camera_to_body[3] == 1.0 &&
                std::abs(extrinsics.camera_origin_in_body.down_m - 0.16) <
                    1.0e-12,
        "Gazebo extrinsics must preserve the measured downward mount");
}

void unsupported_frames_are_rejected() {
    std::istringstream input{
        R"({
          "schema_version": 1,
          "frame_from": "camera_optical",
          "frame_to": "local_ned",
          "rotation_camera_to_body": [
            [1, 0, 0], [0, 1, 0], [0, 0, 1]
          ],
          "camera_origin_in_body_m": {
            "forward": 0, "right": 0, "down": 0
          }
        })"};
    bool rejected = false;
    try {
        static_cast<void>(
            onboard_autonomy::mission::cv::CameraExtrinsicsLoader::from_stream(
                input));
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    require(rejected, "loader must reject unexpected coordinate frames");
}

} // namespace

void run_camera_extrinsics_loader_tests() {
    gazebo_mount_loads_body_frd_transform();
    unsupported_frames_are_rejected();
}
