#include "onboard_autonomy/bootstrap/MissionRuntime.hpp"

#include "onboard_autonomy/hardware/camera/GStreamerCameraSource.hpp"
#include "onboard_autonomy/hardware/camera/RpicamCameraSource.hpp"
#include "onboard_autonomy/hardware/transport/TransportFactory.hpp"
#include "onboard_autonomy/mission/CompanionApplication.hpp"

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
            .sensor_mode = rpicam->sensor_mode,
        });
    }
    const auto& gstreamer = std::get<GStreamerMissionSource>(camera.source);
    return hardware::camera::make_gstreamer_camera_source({
        .width = camera.frame_width,
        .height = camera.frame_height,
        .udp_port = gstreamer.udp_port,
    });
}

} // namespace

class MissionRuntime::Impl {
  public:
    explicit Impl(const MissionRuntimeConfig& config)
        : transport_(make_transport(config)),
          camera_source_(make_camera_source(config)) {
        // CompanionApplication stores non-owning adapter pointers. The
        // runtime owns every adapter and destroys the application first.
        application_ =
            std::make_unique<mission::CompanionApplication>(*transport_,
                mission::CompanionApplicationOptions{
                    .camera_source = camera_source_.get(),
                    .simulated_wind = config.simulated_wind,
                });
    }

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
