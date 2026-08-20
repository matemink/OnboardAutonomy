#include "onboard_autonomy/mission/cv/extrinsics/CameraExtrinsicsLoader.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>

namespace onboard_autonomy::mission::cv {
namespace {

using Json = nlohmann::json;

mission::CameraExtrinsics parse_document(const Json& document) {
    if (document.at("schema_version").get<std::uint32_t>() != 1U ||
        document.at("frame_from").get<std::string>() != "camera_optical" ||
        document.at("frame_to").get<std::string>() != "body_frd") {
        throw std::runtime_error(
            "unsupported camera extrinsics schema or coordinate frames");
    }

    const auto rows = document.at("rotation_camera_to_body")
                          .get<std::array<std::array<double, 3>, 3>>();
    const auto& translation = document.at("camera_origin_in_body_m");
    mission::CameraExtrinsics extrinsics{
        .rotation_camera_to_body =
            {
                rows[0][0],
                rows[0][1],
                rows[0][2],
                rows[1][0],
                rows[1][1],
                rows[1][2],
                rows[2][0],
                rows[2][1],
                rows[2][2],
            },
        .camera_origin_in_body =
            {
                .forward_m = translation.at("forward").get<double>(),
                .right_m = translation.at("right").get<double>(),
                .down_m = translation.at("down").get<double>(),
            },
    };
    mission::validate_camera_extrinsics(extrinsics);
    return extrinsics;
}

} // namespace

mission::CameraExtrinsics CameraExtrinsicsLoader::from_file(
    const std::filesystem::path& path) {
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error(
            "cannot open camera extrinsics: " + path.string());
    }
    return from_stream(input);
}

mission::CameraExtrinsics CameraExtrinsicsLoader::from_stream(
    std::istream& input) {
    try {
        return parse_document(Json::parse(input));
    } catch (const nlohmann::json::exception& error) {
        throw std::runtime_error(
            "invalid camera extrinsics JSON: " + std::string(error.what()));
    }
}

} // namespace onboard_autonomy::mission::cv
