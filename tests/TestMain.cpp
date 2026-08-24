#include "TestCases.hpp"

#include <exception>
#include <iostream>

int main() {
    try {
        run_vehicle_state_tests();
        run_companion_application_tests();
        run_json_diagnostic_sink_tests();
        run_mavlink_decoder_tests();
        run_mavlink_encoder_tests();
        run_telemetry_stream_configurator_tests();
        run_target_tracker_tests();
        run_aerial_target_tracker_tests();
        run_target_transform_tests();
        run_console_view_tests();
        run_board_type_catalog_tests();
        run_camera_monitor_tests();
        run_command_line_tests();
        run_companion_link_failsafe_tests();
        run_vision_monitor_tests();
        run_camera_calibration_loader_tests();
        run_camera_extrinsics_loader_tests();
        run_transport_tests();
        run_motion_safety_policy_tests();
        run_autonomy_core_tests();
        run_flight_startup_controller_tests();
        run_autonomy_runtime_tests();
        run_yolox_image_preprocessor_tests();
        run_yolox_output_decoder_tests();
        std::cout << "All OnboardAutonomy tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failure: " << error.what() << '\n';
        return 1;
    }
}
