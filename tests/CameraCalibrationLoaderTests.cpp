#include "TestCases.hpp"

#include "onboard_autonomy/mission/cv/calibration/CameraCalibrationLoader.hpp"

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

void physical_calibration_loads_exact_values() {
    const auto path = std::filesystem::path{ONBOARD_AUTONOMY_SOURCE_DIR} /
                      "config" / "camera-module-3-wide-640x480.json";
    const auto calibration =
        onboard_autonomy::mission::cv::CameraCalibrationLoader::from_file(path);

    require(calibration.camera_model == "imx708_wide" &&
                calibration.image_width == 640U &&
                calibration.image_height == 480U &&
                std::abs(calibration.fx_px - 543.625989446945) < 1.0e-9 &&
                std::abs(calibration.distortion[4] - 0.4749880305495665) <
                    1.0e-12,
        "loader must preserve the measured physical calibration");
}

std::string calibration_json(const std::string& result,
    const double matrix_fx) {
    return "{"
           "\"schema_version\":1,"
           "\"result\":\"" +
           result +
           "\","
           "\"model\":\"opencv_pinhole_brown_conrady_5\","
           "\"camera\":{"
           "\"model\":\"test-camera\","
           "\"width\":640,\"height\":480,"
           "\"focus_mode\":\"manual\","
           "\"lens_position\":\"default\"},"
           "\"intrinsics\":{"
           "\"fx_px\":500.0,\"fy_px\":501.0,"
           "\"cx_px\":320.0,\"cy_px\":240.0,"
           "\"camera_matrix\":[[" +
           std::to_string(matrix_fx) +
           ",0,320],[0,501,240],[0,0,1]]},"
           "\"distortion\":{"
           "\"coefficient_order\":[\"k1\",\"k2\","
           "\"p1\",\"p2\",\"k3\"],"
           "\"coefficients\":[0.1,-0.2,0.0,0.0,0.3]},"
           "\"quality\":{\"checks\":{"
           "\"minimum_views_reached\":true,"
           "\"rms_error_within_limit\":true,"
           "\"every_view_error_within_limit\":true}}"
           "}";
}

void failed_quality_gate_is_rejected() {
    std::istringstream input{calibration_json("FAIL", 500.0)};
    bool rejected = false;
    try {
        static_cast<void>(
            onboard_autonomy::mission::cv::CameraCalibrationLoader::from_stream(
                input));
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    require(rejected,
        "runtime must reject calibration that failed quality gates");
}

void inconsistent_camera_matrix_is_rejected() {
    std::istringstream input{calibration_json("PASS", 499.0)};
    bool rejected = false;
    try {
        static_cast<void>(
            onboard_autonomy::mission::cv::CameraCalibrationLoader::from_stream(
                input));
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    require(rejected, "redundant matrix and scalar intrinsics must agree");
}

} // namespace

void run_camera_calibration_loader_tests() {
    physical_calibration_loads_exact_values();
    failed_quality_gate_is_rejected();
    inconsistent_camera_matrix_is_rejected();
}
