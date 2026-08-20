#include "onboard_autonomy/bootstrap/MissionRuntime.hpp"

#include "onboard_autonomy/adapters/camera/GStreamerCameraSource.hpp"
#include "onboard_autonomy/adapters/camera/RpicamCameraSource.hpp"
#include "onboard_autonomy/adapters/transport/TransportFactory.hpp"
#include "onboard_autonomy/adapters/vision/AprilTagTargetDetector.hpp"
#include "onboard_autonomy/adapters/vision/CameraCalibrationLoader.hpp"
#include "onboard_autonomy/adapters/vision/CameraExtrinsicsLoader.hpp"
#include "onboard_autonomy/application/CompanionApplication.hpp"
#include "onboard_autonomy/application/MotionSafetyPolicy.hpp"

#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <variant>

namespace onboard_autonomy::bootstrap {
namespace {

std::unique_ptr<application::ports::Transport> make_transport_for(
    const MissionConnection& connection) {
    if (const auto* udp = std::get_if<UdpMissionConnection>(&connection)) {
        return adapters::transport::make_udp_transport(udp->bind_address,
            udp->port);
    }
    const auto& serial = std::get<SerialMissionConnection>(connection);
    return adapters::transport::make_serial_transport(serial.device,
        serial.baud_rate);
}

std::unique_ptr<application::ports::Transport> make_transport(
    const MissionRuntimeConfig& config) {
    return make_transport_for(config.connection);
}

std::unique_ptr<application::ports::CameraSource> make_camera_source(
    const MissionRuntimeConfig& config) {
    const auto& configured_camera = config.camera;
    if (!configured_camera.has_value()) {
        return nullptr;
    }
    const auto& camera = *configured_camera;
    if (const auto* rpicam = std::get_if<RpicamMissionSource>(&camera.source)) {
        return adapters::camera::make_rpicam_camera_source({
            .width = camera.frame_width,
            .height = camera.frame_height,
            .frames_per_second = rpicam->frames_per_second,
        });
    }
    const auto& gstreamer = std::get<GStreamerMissionSource>(camera.source);
    return adapters::camera::make_gstreamer_camera_source({
        .width = camera.frame_width,
        .height = camera.frame_height,
        .udp_port = gstreamer.udp_port,
    });
}

std::unique_ptr<application::ports::TargetDetector> make_target_detector(
    const MissionRuntimeConfig& config) {
    const auto& configured_camera = config.camera;
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
    const MissionRuntimeConfig& config) {
    const auto& camera = config.camera;
    if (!camera.has_value() || !camera->apriltag.has_value() ||
        camera->apriltag->extrinsics_file.empty()) {
        return std::nullopt;
    }
    return adapters::vision::CameraExtrinsicsLoader::from_file(
        camera->apriltag->extrinsics_file);
}

application::MotionSafetyDecision evaluate_safety(
    const MissionRuntimeConfig& config) {
    const auto decision = application::evaluate_motion_safety(
        config.environment == MissionEnvironment::simulation
            ? application::RuntimeEnvironment::sitl
            : application::RuntimeEnvironment::hardware_or_unknown,
        std::holds_alternative<UdpMissionConnection>(config.connection)
            ? application::MavlinkTransport::udp
            : application::MavlinkTransport::serial,
        config.motion_commands_requested);
    if (!decision.configuration_valid) {
        throw std::invalid_argument(std::string(decision.reason));
    }
    return decision;
}

} // namespace

class MissionRuntime::Impl {
  public:
    explicit Impl(const MissionRuntimeConfig& config)
        : safety_(evaluate_safety(config)),
          camera_extrinsics_(load_camera_extrinsics(config)),
          target_detector_(make_target_detector(config)),
          transport_(make_transport(config)),
          camera_source_(make_camera_source(config)) {
        // CompanionApplication stores non-owning adapter pointers. The
        // runtime owns every adapter and destroys the application first.
        application_ =
            std::make_unique<application::CompanionApplication>(*transport_,
                application::CompanionApplicationOptions{
                    .flight_startup =
                        {
                            .enabled = config.autonomous,
                            .takeoff_altitude_m = application::
                                FlightStartupConfig::kDefaultTakeoffAltitudeM,
                        },
                    .autonomy_runtime =
                        {
                            .enabled = config.autonomous,
                        },
                    .motion_commands_allowed = safety_.motion_commands_allowed,
                    .camera_source = camera_source_.get(),
                    .target_detector = target_detector_.get(),
                    .camera_extrinsics = camera_extrinsics_,
                    .simulated_wind = config.simulated_wind,
                });
    }

    application::MotionSafetyDecision safety_;
    std::optional<domain::CameraExtrinsics> camera_extrinsics_;
    std::unique_ptr<application::ports::TargetDetector> target_detector_;
    std::unique_ptr<application::ports::Transport> transport_;
    std::unique_ptr<application::ports::CameraSource> camera_source_;
    std::unique_ptr<application::CompanionApplication> application_;
};

MissionRuntime::MissionRuntime(const MissionRuntimeConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

MissionRuntime::~MissionRuntime() = default;

application::CompanionApplication& MissionRuntime::application() {
    return *impl_->application_;
}

application::ports::Transport& MissionRuntime::transport() {
    return *impl_->transport_;
}

} // namespace onboard_autonomy::bootstrap
