#include "onboard_autonomy/bootstrap/CompanionSystem.hpp"

#include "onboard_autonomy/adapters/ardupilot/BoardTypeCatalog.hpp"
#include "onboard_autonomy/adapters/camera/GStreamerCameraSource.hpp"
#include "onboard_autonomy/adapters/camera/RpicamCameraSource.hpp"
#include "onboard_autonomy/adapters/preview/HttpCameraPreviewServer.hpp"
#include "onboard_autonomy/adapters/transport/TransportFactory.hpp"
#include "onboard_autonomy/adapters/vision/AprilTagTargetDetector.hpp"
#include "onboard_autonomy/adapters/vision/CameraCalibrationLoader.hpp"
#include "onboard_autonomy/adapters/vision/CameraExtrinsicsLoader.hpp"
#include "onboard_autonomy/application/CompanionApplication.hpp"
#include "onboard_autonomy/application/MotionSafetyPolicy.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace onboard_autonomy::bootstrap {
namespace {

const std::optional<presentation::cli::CameraOptions>& camera_options(
    const presentation::cli::CommandLineOptions& options) {
    return std::visit(
        [](const auto& launch)
            -> const std::optional<presentation::cli::CameraOptions>& {
            return launch.camera;
        },
        options);
}

const presentation::cli::AutonomyOptions& autonomy_options(
    const presentation::cli::CommandLineOptions& options) {
    return std::visit(
        [](const auto& launch) -> const presentation::cli::AutonomyOptions& {
            return launch.autonomy;
        },
        options);
}

const presentation::cli::OperatorInterfaceOptions& operator_options(
    const presentation::cli::CommandLineOptions& options) {
    return std::visit(
        [](const auto& launch)
            -> const presentation::cli::OperatorInterfaceOptions& {
            return launch.operator_interface;
        },
        options);
}

std::optional<application::SimulatedWindProfile> simulated_wind(
    const presentation::cli::CommandLineOptions& options) {
    const auto* simulation =
        std::get_if<presentation::cli::SimulationLaunchOptions>(&options);
    return simulation == nullptr ? std::nullopt : simulation->wind;
}

std::unique_ptr<adapters::ardupilot::BoardTypeCatalog> load_board_type_catalog(
    const presentation::cli::CommandLineOptions& options,
    const std::filesystem::path& executable) {
    std::vector<std::filesystem::path> candidates;
    const auto& override_file = operator_options(options).board_types_file;
    if (!override_file.empty()) {
        candidates.emplace_back(override_file);
    } else {
        candidates.push_back((executable.parent_path() / ".." / "share" /
                              "onboard_autonomy" / "ardupilot-board-types.txt")
                                 .lexically_normal());
        candidates.emplace_back("third_party/ardupilot/board_types.txt");
    }

    for (const auto& candidate : candidates) {
        std::error_code error;
        if (std::filesystem::is_regular_file(candidate, error)) {
            return std::make_unique<adapters::ardupilot::BoardTypeCatalog>(
                adapters::ardupilot::BoardTypeCatalog::from_file(candidate));
        }
    }

    if (!override_file.empty()) {
        throw std::runtime_error(
            "board type table not found: " + override_file);
    }
    return nullptr;
}

std::filesystem::path find_camera_preview_page(
    const std::filesystem::path& executable) {
    const std::vector<std::filesystem::path> candidates{
        (executable.parent_path() / ".." / "share" / "onboard_autonomy" /
            "camera-preview.html")
            .lexically_normal(),
        "assets/camera-preview/index.html",
    };

    for (const auto& candidate : candidates) {
        std::error_code error;
        if (std::filesystem::is_regular_file(candidate, error)) {
            return candidate;
        }
    }
    throw std::runtime_error("camera preview page was not found");
}

std::unique_ptr<application::ports::Transport> make_transport_for(
    const presentation::cli::HardwareLaunchOptions& options) {
    if (const auto* udp = std::get_if<presentation::cli::UdpConnectionOptions>(
            &options.connection)) {
        return adapters::transport::make_udp_transport(udp->bind_address,
            udp->port);
    }
    const auto& serial = std::get<presentation::cli::SerialConnectionOptions>(
        options.connection);
    return adapters::transport::make_serial_transport(serial.device,
        serial.baud_rate);
}

std::unique_ptr<application::ports::Transport> make_transport_for(
    const presentation::cli::SimulationLaunchOptions& options) {
    return adapters::transport::make_udp_transport(
        options.connection.bind_address,
        options.connection.port);
}

std::unique_ptr<application::ports::Transport> make_transport(
    const presentation::cli::CommandLineOptions& options) {
    return std::visit(
        [](const auto& launch) { return make_transport_for(launch); },
        options);
}

std::unique_ptr<application::ports::CameraSource> make_camera_source(
    const presentation::cli::CommandLineOptions& options) {
    const auto& configured_camera = camera_options(options);
    if (!configured_camera.has_value()) {
        return nullptr;
    }
    const auto& camera = *configured_camera;
    if (const auto* rpicam =
            std::get_if<presentation::cli::RpicamOptions>(&camera.source)) {
        return adapters::camera::make_rpicam_camera_source({
            .width = camera.frame_width,
            .height = camera.frame_height,
            .frames_per_second = rpicam->frames_per_second,
        });
    }
    const auto& gstreamer =
        std::get<presentation::cli::GStreamerCameraOptions>(camera.source);
    return adapters::camera::make_gstreamer_camera_source({
        .width = camera.frame_width,
        .height = camera.frame_height,
        .udp_port = gstreamer.udp_port,
    });
}

std::unique_ptr<application::ports::TargetDetector> make_target_detector(
    const presentation::cli::CommandLineOptions& options) {
    const auto& configured_camera = camera_options(options);
    if (!configured_camera.has_value() ||
        !configured_camera->apriltag.has_value()) {
        return nullptr;
    }

    const auto& camera = *configured_camera;
    const auto& apriltag = *camera.apriltag;
    adapters::vision::AprilTagDetectorConfig detector_config;
    if (!apriltag.calibration_file.empty()) {
        const auto tag_size = apriltag.tag_size_m;
        if (!tag_size.has_value()) {
            throw std::invalid_argument(
                "camera calibration requires AprilTag size");
        }
        auto calibration = adapters::vision::CameraCalibrationLoader::from_file(
            apriltag.calibration_file);
        if (calibration.image_width != camera.frame_width ||
            calibration.image_height != camera.frame_height) {
            throw std::invalid_argument(
                "camera runtime resolution does not match calibration");
        }
        detector_config.pose = adapters::vision::AprilTagPoseConfig{
            .calibration = std::move(calibration),
            .tag_size_m = tag_size.value(),
        };
    }
    return adapters::vision::make_apriltag_target_detector(detector_config);
}

std::optional<domain::CameraExtrinsics> load_camera_extrinsics(
    const presentation::cli::CommandLineOptions& options) {
    const auto& camera = camera_options(options);
    if (!camera.has_value() || !camera->apriltag.has_value() ||
        camera->apriltag->extrinsics_file.empty()) {
        return std::nullopt;
    }
    return adapters::vision::CameraExtrinsicsLoader::from_file(
        camera->apriltag->extrinsics_file);
}

std::unique_ptr<application::ports::CameraPreviewSink> make_camera_preview(
    const presentation::cli::CommandLineOptions& options,
    const std::filesystem::path& executable) {
    const auto& camera = camera_options(options);
    if (!camera.has_value() || !camera->preview.has_value()) {
        return nullptr;
    }
    return adapters::preview::make_http_camera_preview_server({
        .bind_address = "0.0.0.0",
        .port = camera->preview->port,
        .maximum_frames_per_second = adapters::preview::
            HttpCameraPreviewConfig::kDefaultMaximumFramesPerSecond,
        .page_file = find_camera_preview_page(executable),
    });
}

application::MotionSafetyDecision evaluate_safety(
    const presentation::cli::CommandLineOptions& options) {
    const auto& autonomy = autonomy_options(options);
    const auto& operator_interface = operator_options(options);
    const auto* hardware =
        std::get_if<presentation::cli::HardwareLaunchOptions>(&options);
    const auto decision = application::evaluate_motion_safety(
        hardware == nullptr
            ? application::RuntimeEnvironment::sitl
            : application::RuntimeEnvironment::hardware_or_unknown,
        hardware == nullptr ||
                std::holds_alternative<presentation::cli::UdpConnectionOptions>(
                    hardware->connection)
            ? application::MavlinkTransport::udp
            : application::MavlinkTransport::serial,
        autonomy.enabled || operator_interface.interactive);
    if (!decision.configuration_valid) {
        throw std::invalid_argument(std::string(decision.reason));
    }
    return decision;
}

} // namespace

class CompanionSystem::Impl {
  public:
    Impl(const presentation::cli::CommandLineOptions& options,
        const std::filesystem::path& executable)
        : safety_(evaluate_safety(options)),
          camera_extrinsics_(load_camera_extrinsics(options)),
          board_type_catalog_(
              operator_options(options).json_output
                  ? nullptr
                  : load_board_type_catalog(options, executable)),
          target_detector_(make_target_detector(options)),
          transport_(make_transport(options)),
          camera_source_(make_camera_source(options)),
          camera_preview_(make_camera_preview(options, executable)) {
        const auto& autonomy = autonomy_options(options);
        // CompanionApplication stores non-owning adapter pointers. The
        // system owns every adapter and destroys the application first.
        application_ =
            std::make_unique<application::CompanionApplication>(*transport_,
                application::CompanionApplicationOptions{
                    .flight_startup =
                        {
                            .enabled = autonomy.enabled,
                            .takeoff_altitude_m = application::
                                FlightStartupConfig::kDefaultTakeoffAltitudeM,
                        },
                    .autonomy_runtime =
                        {
                            .enabled = autonomy.enabled,
                        },
                    .motion_commands_allowed = safety_.motion_commands_allowed,
                    .camera_source = camera_source_.get(),
                    .target_detector = target_detector_.get(),
                    .camera_preview_sink = camera_preview_.get(),
                    .camera_extrinsics = camera_extrinsics_,
                    .simulated_wind = simulated_wind(options),
                });
    }

    // Validate pure configuration before opening serial or camera resources.
    [[nodiscard]] const presentation::BoardTypeResolver*
    board_type_resolver() const {
        return board_type_catalog_.get();
    }

    application::MotionSafetyDecision safety_;
    std::optional<domain::CameraExtrinsics> camera_extrinsics_;
    std::unique_ptr<adapters::ardupilot::BoardTypeCatalog> board_type_catalog_;
    std::unique_ptr<application::ports::TargetDetector> target_detector_;
    std::unique_ptr<application::ports::Transport> transport_;
    std::unique_ptr<application::ports::CameraSource> camera_source_;
    std::unique_ptr<application::ports::CameraPreviewSink> camera_preview_;
    std::unique_ptr<application::CompanionApplication> application_;
};

CompanionSystem::CompanionSystem(
    const presentation::cli::CommandLineOptions& options,
    const std::filesystem::path& executable)
    : impl_(std::make_unique<Impl>(options, executable)) {}

CompanionSystem::~CompanionSystem() = default;

application::CompanionApplication& CompanionSystem::application() {
    return *impl_->application_;
}

application::ports::Transport& CompanionSystem::transport() {
    return *impl_->transport_;
}

const presentation::BoardTypeResolver*
CompanionSystem::board_type_resolver() const {
    return impl_->board_type_resolver();
}

} // namespace onboard_autonomy::bootstrap
