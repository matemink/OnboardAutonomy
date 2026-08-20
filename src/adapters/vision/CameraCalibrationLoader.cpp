#include "onboard_autonomy/adapters/vision/CameraCalibrationLoader.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>

namespace onboard_autonomy::adapters::vision {
namespace {

using Json = nlohmann::json;

constexpr double kMatrixTolerance = 1.0e-9;

void require_quality_pass(const Json& document) {
    const auto& checks = document.at("quality").at("checks");
    if (document.at("result").get<std::string>() != "PASS" ||
        !checks.at("minimum_views_reached").get<bool>() ||
        !checks.at("rms_error_within_limit").get<bool>() ||
        !checks.at("every_view_error_within_limit").get<bool>()) {
        throw std::runtime_error(
            "camera calibration did not pass its quality gates");
    }
}

void require_supported_schema(const Json& document) {
    if (document.at("schema_version").get<std::uint32_t>() != 1U ||
        document.at("model").get<std::string>() !=
            "opencv_pinhole_brown_conrady_5") {
        throw std::runtime_error(
            "unsupported camera calibration schema or model");
    }

    const std::array<std::string,
        domain::CameraCalibration::kDistortionCoefficientCount>
        expected_order{"k1", "k2", "p1", "p2", "k3"};
    if (document.at("distortion")
            .at("coefficient_order")
            .get<std::array<std::string,
                domain::CameraCalibration::kDistortionCoefficientCount>>() !=
        expected_order) {
        throw std::runtime_error(
            "unsupported camera distortion coefficient order");
    }
}

void require_consistent_matrix(const Json& intrinsics,
    const domain::CameraCalibration& calibration) {
    const auto matrix = intrinsics.at("camera_matrix")
                            .get<std::array<std::array<double, 3>, 3>>();
    const auto close = [](const double left, const double right) {
        return std::abs(left - right) <= kMatrixTolerance;
    };

    if (!close(matrix[0][0], calibration.fx_px) ||
        !close(matrix[1][1], calibration.fy_px) ||
        !close(matrix[0][2], calibration.cx_px) ||
        !close(matrix[1][2], calibration.cy_px) || !close(matrix[0][1], 0.0) ||
        !close(matrix[1][0], 0.0) || !close(matrix[2][0], 0.0) ||
        !close(matrix[2][1], 0.0) || !close(matrix[2][2], 1.0)) {
        throw std::runtime_error(
            "camera matrix disagrees with scalar intrinsics");
    }
}

domain::CameraCalibration parse_document(const Json& document) {
    require_supported_schema(document);
    require_quality_pass(document);

    const auto& camera = document.at("camera");
    const auto& intrinsics = document.at("intrinsics");
    domain::CameraCalibration calibration{
        .camera_model = camera.at("model").get<std::string>(),
        .image_width = camera.at("width").get<std::uint32_t>(),
        .image_height = camera.at("height").get<std::uint32_t>(),
        .focus_mode = camera.at("focus_mode").get<std::string>(),
        .lens_position = camera.at("lens_position").get<std::string>(),
        .fx_px = intrinsics.at("fx_px").get<double>(),
        .fy_px = intrinsics.at("fy_px").get<double>(),
        .cx_px = intrinsics.at("cx_px").get<double>(),
        .cy_px = intrinsics.at("cy_px").get<double>(),
        .distortion =
            document.at("distortion")
                .at("coefficients")
                .get<std::array<double,
                    domain::CameraCalibration::kDistortionCoefficientCount>>(),
    };
    require_consistent_matrix(intrinsics, calibration);
    domain::validate_camera_calibration(calibration);
    return calibration;
}

} // namespace

domain::CameraCalibration CameraCalibrationLoader::from_file(
    const std::filesystem::path& path) {
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error(
            "cannot open camera calibration: " + path.string());
    }
    return from_stream(input);
}

domain::CameraCalibration CameraCalibrationLoader::from_stream(
    std::istream& input) {
    try {
        return parse_document(Json::parse(input));
    } catch (const nlohmann::json::exception& error) {
        throw std::runtime_error(
            "invalid camera calibration JSON: " + std::string(error.what()));
    }
}

} // namespace onboard_autonomy::adapters::vision
