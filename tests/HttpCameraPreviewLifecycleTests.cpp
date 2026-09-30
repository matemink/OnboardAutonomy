#include "onboard_autonomy/diagnostics/preview/HttpCameraPreviewServer.hpp"

#include <httplib.h>

#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

onboard_autonomy::diagnostics::preview::HttpCameraPreviewConfig config_for(
    const int port) {
    return {.bind_address = "127.0.0.1",
        .port = static_cast<std::uint16_t>(port),
        .page_file = std::filesystem::path{ONBOARD_AUTONOMY_SOURCE_DIR} /
                     "assets/camera-preview/index.html"};
}
} // namespace

int main() {
    try {
        int available_port = -1;
        {
            httplib::Server reservation;
            reservation.set_socket_options([](const socket_t) {});
            available_port = reservation.bind_to_any_port("127.0.0.1");
            require(available_port > 0, "could not reserve a test port");
            std::jthread listener{
                [&reservation] { reservation.listen_after_bind(); }};
            reservation.wait_until_ready();
            struct StopBeforeJoin {
                httplib::Server& server;
                ~StopBeforeJoin() { server.stop(); }
            } stop{reservation};
            auto preview = onboard_autonomy::diagnostics::preview::
                make_http_camera_preview_server(config_for(available_port));
            require(preview->description().find("listen failed") !=
                        std::string::npos,
                "occupied port must report failure without hanging startup or "
                "shutdown");
        }
        for (int iteration = 0; iteration < 20; ++iteration) {
            auto preview = onboard_autonomy::diagnostics::preview::
                make_http_camera_preview_server(config_for(available_port));
            require(preview->description().find("listen failed") ==
                        std::string::npos,
                "a released port must be reusable across immediate shutdowns");
            if (iteration == 0) {
                using namespace onboard_autonomy;
                const mission::ports::CameraFrame frame{
                    .sequence = 1,
                    .width = 2,
                    .height = 2,
                    .yuv420 = {16, 16, 16, 16, 128, 128},
                    .captured_at = std::nullopt,
                    .received_at = {},
                };
                const std::vector<mission::TargetObservation> objects{{
                    .id = 7,
                    .family = "object",
                    .center = {.x_px = 1.0, .y_px = 1.0},
                    .corners = {},
                    .confidence_percent = 85.0,
                }};
                preview->publish(
                    diagnostics::preview::CameraPreviewStream::forward,
                    frame,
                    objects);
                httplib::Client client{"127.0.0.1", available_port};
                const auto camera = client.Get("/api/frame/forward");
                require(camera && camera->status == 200 &&
                            camera->body.size() == 6,
                    "preview must serve the published I420 frame");
                require(!camera->has_header("X-OnboardAutonomy-Target-Track"),
                    "preview must not publish a removed marker track");
                const auto detections =
                    camera->get_header_value("X-OnboardAutonomy-Targets");
                require(detections.find("confidence_percent") !=
                                std::string::npos &&
                            detections.find("pose") == std::string::npos &&
                            detections.find("corrected_bits") ==
                                std::string::npos,
                    "preview must expose object confidence without marker "
                    "fields");
                const auto result = client.Get("/");
                require(result && result->status == 200 &&
                            !result->body.empty(),
                    "a successfully constructed preview must serve its page");
            }
        }
        std::cout << "HTTP preview lifecycle tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
