#include "onboard_autonomy/bootstrap/MissionRuntime.hpp"

#include "onboard_autonomy/hardware/camera/GStreamerCameraSource.hpp"
#include "onboard_autonomy/hardware/camera/RpicamCameraSource.hpp"
#include "onboard_autonomy/hardware/transport/TransportFactory.hpp"
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
                        config.environment == MissionEnvironment::simulation &&
                        config.aerial_tracking_allowed,
                    .camera_source = camera_source_.get(),
                    .simulated_wind = config.simulated_wind,
                });
    }

    mission::MotionSafetyDecision safety_;
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
