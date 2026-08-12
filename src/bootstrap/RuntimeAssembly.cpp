#include "onboard_autonomy/bootstrap/RuntimeAssembly.hpp"

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
#include <vector>

namespace onboard_autonomy::bootstrap {
namespace {

std::optional<adapters::ardupilot::BoardTypeCatalog>
load_board_type_catalog(
    const presentation::cli::CommandLineOptions& options,
    const std::filesystem::path& executable
) {
    std::vector<std::filesystem::path> candidates;
    if (!options.board_types_file.empty()) {
        candidates.emplace_back(options.board_types_file);
    } else {
        candidates.push_back(
            (
                executable.parent_path() / ".." / "share" /
                "onboard_autonomy" / "ardupilot-board-types.txt"
            ).lexically_normal()
        );
        candidates.emplace_back("third_party/ardupilot/board_types.txt");
    }

    for (const auto& candidate : candidates) {
        std::error_code error;
        if (std::filesystem::is_regular_file(candidate, error)) {
            return adapters::ardupilot::BoardTypeCatalog::from_file(
                candidate
            );
        }
    }

    if (!options.board_types_file.empty()) {
        throw std::runtime_error(
            "board type table not found: " + options.board_types_file
        );
    }
    return std::nullopt;
}

std::filesystem::path find_camera_preview_page(
    const std::filesystem::path& executable
) {
    const std::vector<std::filesystem::path> candidates{
        (
            executable.parent_path() / ".." / "share" /
            "onboard_autonomy" / "camera-preview.html"
        ).lexically_normal(),
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

std::unique_ptr<application::ports::Transport> make_transport(
    const presentation::cli::CommandLineOptions& options
) {
    if (options.transport == presentation::cli::TransportBackend::udp) {
        return adapters::transport::make_udp_transport(
            options.udp_bind,
            options.udp_port
        );
    }
    return adapters::transport::make_serial_transport(
        options.serial_device,
        options.baud_rate
    );
}

std::unique_ptr<application::ports::CameraSource> make_camera_source(
    const presentation::cli::CommandLineOptions& options
) {
    if (!options.camera_enabled) {
        return nullptr;
    }
    if (options.camera_backend == presentation::cli::CameraBackend::rpicam) {
        return adapters::camera::make_rpicam_camera_source(
            {
                .width = options.camera_width,
                .height = options.camera_height,
                .frames_per_second = options.camera_fps,
            }
        );
    }
    return adapters::camera::make_gstreamer_camera_source(
        {
            .width = options.camera_width,
            .height = options.camera_height,
            .udp_port = options.camera_udp_port,
        }
    );
}

std::unique_ptr<application::ports::TargetDetector> make_target_detector(
    const presentation::cli::CommandLineOptions& options
) {
    if (!options.apriltag_enabled) {
        return nullptr;
    }

    adapters::vision::AprilTagDetectorConfig detector_config;
    if (!options.camera_calibration_file.empty()) {
        auto calibration =
            adapters::vision::CameraCalibrationLoader::from_file(
                options.camera_calibration_file
            );
        if (calibration.image_width != options.camera_width ||
            calibration.image_height != options.camera_height) {
            throw std::invalid_argument(
                "camera runtime resolution does not match calibration"
            );
        }
        detector_config.pose = adapters::vision::AprilTagPoseConfig{
            .calibration = std::move(calibration),
            .tag_size_m = *options.apriltag_tag_size_m,
        };
    }
    return adapters::vision::make_apriltag_target_detector(
        detector_config
    );
}

std::optional<domain::CameraExtrinsics> load_camera_extrinsics(
    const presentation::cli::CommandLineOptions& options
) {
    if (options.camera_extrinsics_file.empty()) {
        return std::nullopt;
    }
    return adapters::vision::CameraExtrinsicsLoader::from_file(
        options.camera_extrinsics_file
    );
}

std::unique_ptr<application::ports::CameraPreviewSink> make_camera_preview(
    const presentation::cli::CommandLineOptions& options,
    const std::filesystem::path& executable
) {
    if (!options.camera_preview_enabled) {
        return nullptr;
    }
    return adapters::preview::make_http_camera_preview_server(
        {
            .bind_address = "0.0.0.0",
            .port = options.camera_preview_port,
            .maximum_frames_per_second = 10,
            .page_file = find_camera_preview_page(executable),
        }
    );
}

application::MotionSafetyDecision evaluate_safety(
    const presentation::cli::CommandLineOptions& options
) {
    const auto decision = application::evaluate_motion_safety(
        options.sitl_mode
            ? application::RuntimeEnvironment::sitl
            : application::RuntimeEnvironment::hardware_or_unknown,
        options.transport == presentation::cli::TransportBackend::udp
            ? application::MavlinkTransport::udp
            : application::MavlinkTransport::serial,
        options.autonomous || options.interactive
    );
    if (!decision.configuration_valid) {
        throw std::invalid_argument(std::string(decision.reason));
    }
    return decision;
}

}  // namespace

class RuntimeAssembly::Impl {
public:
    Impl(
        const presentation::cli::CommandLineOptions& options,
        const std::filesystem::path& executable
    )
        : safety_(evaluate_safety(options)),
          camera_extrinsics_(load_camera_extrinsics(options)),
          board_type_catalog_(
              options.json_output
                  ? std::nullopt
                  : load_board_type_catalog(options, executable)
          ),
          target_detector_(make_target_detector(options)),
          transport_(make_transport(options)),
          camera_source_(make_camera_source(options)),
          camera_preview_(make_camera_preview(options, executable)) {
        // CompanionApplication stores non-owning adapter pointers. The
        // assembly owns every adapter and destroys the application first.
        application_ = std::make_unique<application::CompanionApplication>(
            *transport_,
            application::CompanionApplicationOptions{
                .flight_startup = {
                    .enabled = options.autonomous,
                    .takeoff_altitude_m = 8.0,
                },
                .autonomy_runtime = {
                    .enabled = options.autonomous,
                },
                .motion_commands_allowed = safety_.motion_commands_allowed,
                .camera_source = camera_source_.get(),
                .target_detector = target_detector_.get(),
                .camera_preview_sink = camera_preview_.get(),
                .camera_extrinsics = camera_extrinsics_,
                .simulated_wind = options.simulated_wind,
            }
        );
    }

    // Validate pure configuration before opening serial or camera resources.
    application::MotionSafetyDecision safety_;
    std::optional<domain::CameraExtrinsics> camera_extrinsics_;
    std::optional<adapters::ardupilot::BoardTypeCatalog>
        board_type_catalog_;
    std::unique_ptr<application::ports::TargetDetector> target_detector_;
    std::unique_ptr<application::ports::Transport> transport_;
    std::unique_ptr<application::ports::CameraSource> camera_source_;
    std::unique_ptr<application::ports::CameraPreviewSink> camera_preview_;
    std::unique_ptr<application::CompanionApplication> application_;
};

RuntimeAssembly::RuntimeAssembly(
    const presentation::cli::CommandLineOptions& options,
    const std::filesystem::path& executable
)
    : impl_(std::make_unique<Impl>(options, executable)) {}

RuntimeAssembly::~RuntimeAssembly() = default;

application::CompanionApplication& RuntimeAssembly::application() {
    return *impl_->application_;
}

application::ports::Transport& RuntimeAssembly::transport() {
    return *impl_->transport_;
}

const presentation::BoardTypeResolver*
RuntimeAssembly::board_type_resolver() const {
    return impl_->board_type_catalog_.has_value()
        ? &*impl_->board_type_catalog_
        : nullptr;
}

}  // namespace onboard_autonomy::bootstrap
