#include "onboard_autonomy/bootstrap/MissionRuntime.hpp"

#include "onboard_autonomy/hardware/camera/GStreamerCameraSource.hpp"
#include "onboard_autonomy/hardware/camera/RpicamCameraSource.hpp"
#include "onboard_autonomy/hardware/transport/TransportFactory.hpp"
#include "onboard_autonomy/mission/cv/detection/AprilTagTargetDetector.hpp"
#include "onboard_autonomy/mission/cv/calibration/CameraCalibrationLoader.hpp"
#include "onboard_autonomy/mission/cv/extrinsics/CameraExtrinsicsLoader.hpp"
#include "onboard_autonomy/mission/CompanionApplication.hpp"
#include "onboard_autonomy/mission/safety/MotionSafetyPolicy.hpp"

#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <variant>

namespace onboard_autonomy::bootstrap {
namespace {

std::unique_ptr<mission::ports::Transport> make_transport_for(
    const MissionConnection& connection) {
    if (const auto* udp = std::get_if<UdpMissionConnection>(&connection)) {
        return hardware::transport::make_udp_transport(udp->bind_address,
            udp->port);
    }
    const auto& serial = std::get<SerialMissionConnection>(connection);
    return hardware::transport::make_serial_transport(serial.device,
        serial.baud_rate);
}

std::unique_ptr<mission::ports::Transport> make_transport(
    const MissionRuntimeConfig& config) {
    return make_transport_for(config.connection);
}

std::unique_ptr<mission::ports::CameraSource> make_camera_source(
    const MissionRuntimeConfig& config) {
    const auto& configured_camera = config.camera;
    if (!configured_camera.has_value()) {
        return nullptr;
    }
    const auto& camera = *configured_camera;
    if (const auto* rpicam = std::get_if<RpicamMissionSource>(&camera.source)) {
        return hardware::camera::make_rpicam_camera_source({
            .width = camera.frame_width,
            .height = camera.frame_height,
            .frames_per_second = rpicam->frames_per_second,
        });
    }
    const auto& gstreamer = std::get<GStreamerMissionSource>(camera.source);
    return hardware::camera::make_gstreamer_camera_source({
        .width = camera.frame_width,
        .height = camera.frame_height,
        .udp_port = gstreamer.udp_port,
    });
}

std::unique_ptr<mission::ports::TargetDetector> make_target_detector(
    const MissionRuntimeConfig& config) {
    const auto& configured_camera = config.camera;
    if (!configured_camera.has_value() ||
        !configured_camera->apriltag.has_value()) {
        return nullptr;
    }

    const auto& camera = *configured_camera;
    const auto& apriltag = *camera.apriltag;
    mission::cv::AprilTagDetectorConfig detector_config;
    if (!apriltag.calibration_file.empty()) {
        const auto tag_size = apriltag.tag_size_m;
        if (!tag_size.has_value()) {
            throw std::invalid_argument(
                "camera calibration requires AprilTag size");
        }
        auto calibration = mission::cv::CameraCalibrationLoader::from_file(
            apriltag.calibration_file);
        if (calibration.image_width != camera.frame_width ||
            calibration.image_height != camera.frame_height) {
            throw std::invalid_argument(
                "camera runtime resolution does not match calibration");
        }
        detector_config.pose = mission::cv::AprilTagPoseConfig{
            .calibration = std::move(calibration),
            .tag_size_m = tag_size.value(),
        };
    }
    return mission::cv::make_apriltag_target_detector(detector_config);
}

std::optional<mission::CameraExtrinsics> load_camera_extrinsics(
    const MissionRuntimeConfig& config) {
    const auto& camera = config.camera;
    if (!camera.has_value() || !camera->apriltag.has_value() ||
        camera->apriltag->extrinsics_file.empty()) {
        return std::nullopt;
    }
    return mission::cv::CameraExtrinsicsLoader::from_file(
        camera->apriltag->extrinsics_file);
}

mission::MotionSafetyDecision evaluate_safety(
    const MissionRuntimeConfig& config) {
    const auto decision = mission::evaluate_motion_safety(
        config.environment == MissionEnvironment::simulation
            ? mission::RuntimeEnvironment::sitl
            : mission::RuntimeEnvironment::hardware_or_unknown,
        std::holds_alternative<UdpMissionConnection>(config.connection)
            ? mission::MavlinkTransport::udp
            : mission::MavlinkTransport::serial,
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
            std::make_unique<mission::CompanionApplication>(*transport_,
                mission::CompanionApplicationOptions{
                    .flight_startup =
                        {
                            .enabled = config.autonomous,
                            .start_automatically = config.start_automatically,
                            .takeoff_altitude_m = mission::FlightStartupConfig::
                                kDefaultTakeoffAltitudeM,
                        },
                    .autonomy_runtime =
                        {
                            .enabled = config.autonomous,
                            .start_automatically = config.start_automatically,
                            .mode = config.autonomy_mode,
                        },
                    .motion_commands_allowed = safety_.motion_commands_allowed,
                    .aerial_tracking_allowed =
                        config.environment == MissionEnvironment::simulation,
                    .camera_source = camera_source_.get(),
                    .target_detector = target_detector_.get(),
                    .camera_extrinsics = camera_extrinsics_,
                    .simulated_wind = config.simulated_wind,
                });
    }

    mission::MotionSafetyDecision safety_;
    std::optional<mission::CameraExtrinsics> camera_extrinsics_;
    std::unique_ptr<mission::ports::TargetDetector> target_detector_;
    std::unique_ptr<mission::ports::Transport> transport_;
    std::unique_ptr<mission::ports::CameraSource> camera_source_;
    std::unique_ptr<mission::CompanionApplication> application_;
};

MissionRuntime::MissionRuntime(const MissionRuntimeConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

MissionRuntime::~MissionRuntime() = default;

mission::CompanionApplication& MissionRuntime::application() {
    return *impl_->application_;
}

mission::ports::Transport& MissionRuntime::transport() {
    return *impl_->transport_;
}

} // namespace onboard_autonomy::bootstrap
