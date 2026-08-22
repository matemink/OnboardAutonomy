cmake_minimum_required(VERSION 3.22)

project(
    OnboardAutonomy
    VERSION 0.1.0
    DESCRIPTION "Onboard autonomy runtime for ArduPilot-based UAVs"
    LANGUAGES C CXX
)

find_package(Threads REQUIRED)

option(
    ONBOARD_AUTONOMY_ENABLE_OPENCV_DNN
    "Enable OpenCV DNN object detection"
    ON
)
if(ONBOARD_AUTONOMY_ENABLE_OPENCV_DNN)
    find_package(OpenCV 4.6 REQUIRED COMPONENTS core dnn imgproc)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

# Generated MAVLink headers are used instead of implementing the protocol.
FetchContent_Declare(
    mavlink_headers
    GIT_REPOSITORY https://github.com/mavlink/c_library_v2.git
    GIT_TAG ee5827fcb834bf9e149dc2317fd444e4d1ef9d2e
    GIT_SHALLOW FALSE
)
FetchContent_MakeAvailable(mavlink_headers)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG 65ee68451d8eb2b5f3a30b410476ab83deb3289b
    GIT_SHALLOW FALSE
)
FetchContent_MakeAvailable(nlohmann_json)

FetchContent_Declare(
    apriltag_source
    GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git
    GIT_TAG 94be783968e5091bcc9972c72c84fd63efce2935
    GIT_SHALLOW FALSE
)

FetchContent_Declare(
    cpp_httplib_source
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG 34b7c846d43e30598f3b748ef5ce23591077cb7e
    GIT_SHALLOW FALSE
)
FetchContent_GetProperties(cpp_httplib_source)
if(NOT cpp_httplib_source_POPULATED)
    FetchContent_Populate(cpp_httplib_source)
endif()
FetchContent_GetProperties(apriltag_source)
if(NOT apriltag_source_POPULATED)
    FetchContent_Populate(apriltag_source)
    set(BUILD_SHARED_LIBS OFF)
    set(BUILD_EXAMPLES OFF)
    set(BUILD_PYTHON_WRAPPER OFF)
    set(onboard_autonomy_build_testing_before_apriltag ${BUILD_TESTING})
    set(BUILD_TESTING OFF)
    add_subdirectory(
        ${apriltag_source_SOURCE_DIR}
        ${apriltag_source_BINARY_DIR}
        EXCLUDE_FROM_ALL
    )
    set(BUILD_TESTING ${onboard_autonomy_build_testing_before_apriltag})
endif()

add_library(onboard_autonomy_public_headers INTERFACE)
target_include_directories(
    onboard_autonomy_public_headers
    INTERFACE
        ${PROJECT_SOURCE_DIR}/include
)

add_library(onboard_autonomy_mavlink_headers INTERFACE)
target_include_directories(
    onboard_autonomy_mavlink_headers
    SYSTEM INTERFACE
        ${mavlink_headers_SOURCE_DIR}
)

function(onboard_autonomy_enable_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(
            ${target}
            PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -Wconversion
                -Wsign-conversion
        )
    endif()
endfunction()

add_library(
    onboard_autonomy_mission_types
    src/mission/cv/calibration/CameraCalibration.cpp
    src/mission/cv/extrinsics/TargetTransform.cpp
    src/mission/flight/VehicleState.cpp
)
target_link_libraries(
    onboard_autonomy_mission_types
    PUBLIC
        onboard_autonomy_public_headers
)
onboard_autonomy_enable_warnings(onboard_autonomy_mission_types)

add_library(onboard_autonomy_mission_transport_port INTERFACE)
target_link_libraries(
    onboard_autonomy_mission_transport_port
    INTERFACE
        onboard_autonomy_public_headers
)

add_library(onboard_autonomy_mission_camera_port INTERFACE)
target_link_libraries(
    onboard_autonomy_mission_camera_port
    INTERFACE
        onboard_autonomy_public_headers
)

add_library(onboard_autonomy_mission_target_detector_port INTERFACE)
target_link_libraries(
    onboard_autonomy_mission_target_detector_port
    INTERFACE
        onboard_autonomy_public_headers
)

add_library(onboard_autonomy_diagnostics_preview_port INTERFACE)
target_link_libraries(
    onboard_autonomy_diagnostics_preview_port
    INTERFACE
        onboard_autonomy_mission_camera_port
        onboard_autonomy_public_headers
)

add_library(
    onboard_autonomy_hardware_camera
    src/hardware/camera/GStreamerCameraSource.cpp
    src/hardware/camera/RpicamCameraSource.cpp
)
target_link_libraries(
    onboard_autonomy_hardware_camera
    PUBLIC
        onboard_autonomy_mission_camera_port
    PRIVATE
        Threads::Threads
)
onboard_autonomy_enable_warnings(onboard_autonomy_hardware_camera)

add_library(
    onboard_autonomy_mission_cv_detection
    src/mission/cv/detection/AprilTagTargetDetector.cpp
    src/mission/cv/detection/CameraGeometry.cpp
    src/mission/cv/detection/OpenCvDnnTargetDetector.cpp
    src/mission/cv/detection/YoloXImagePreprocessor.cpp
    src/mission/cv/detection/YoloXOutputDecoder.cpp
)
target_link_libraries(
    onboard_autonomy_mission_cv_detection
    PUBLIC
        onboard_autonomy_mission_target_detector_port
    PRIVATE
        apriltag
)
if(ONBOARD_AUTONOMY_ENABLE_OPENCV_DNN)
    target_compile_definitions(
        onboard_autonomy_mission_cv_detection
        PRIVATE ONBOARD_AUTONOMY_ENABLE_OPENCV_DNN=1
    )
    target_link_libraries(
        onboard_autonomy_mission_cv_detection
        PRIVATE ${OpenCV_LIBS}
    )
    target_include_directories(
        onboard_autonomy_mission_cv_detection
        SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS}
    )
endif()
target_include_directories(
    onboard_autonomy_mission_cv_detection
    SYSTEM PRIVATE
        ${apriltag_source_SOURCE_DIR}
)
onboard_autonomy_enable_warnings(onboard_autonomy_mission_cv_detection)

add_library(
    onboard_autonomy_mission_cv_geometry
    src/mission/cv/calibration/CameraCalibrationLoader.cpp
    src/mission/cv/extrinsics/CameraExtrinsicsLoader.cpp
)
target_link_libraries(
    onboard_autonomy_mission_cv_geometry
    PUBLIC
        onboard_autonomy_mission_types
        onboard_autonomy_public_headers
    PRIVATE
        nlohmann_json::nlohmann_json
)
onboard_autonomy_enable_warnings(
    onboard_autonomy_mission_cv_geometry
)

add_library(
    onboard_autonomy_diagnostics_preview
    src/diagnostics/preview/HttpCameraPreviewServer.cpp
)
target_link_libraries(
    onboard_autonomy_diagnostics_preview
    PUBLIC
        onboard_autonomy_diagnostics_preview_port
    PRIVATE
        Threads::Threads
)
target_include_directories(
    onboard_autonomy_diagnostics_preview
    SYSTEM PRIVATE
        ${cpp_httplib_source_SOURCE_DIR}
)
onboard_autonomy_enable_warnings(onboard_autonomy_diagnostics_preview)

add_library(
    onboard_autonomy_hardware_transport
    src/hardware/transport/PosixSerialTransport.cpp
    src/hardware/transport/UdpTransport.cpp
)
target_link_libraries(
    onboard_autonomy_hardware_transport
    PUBLIC
        onboard_autonomy_mission_transport_port
)
if(WIN32)
    target_link_libraries(onboard_autonomy_hardware_transport PRIVATE ws2_32)
endif()
onboard_autonomy_enable_warnings(onboard_autonomy_hardware_transport)

add_library(
    onboard_autonomy_hardware_mavlink
    src/hardware/mavlink/MavlinkDecoder.cpp
    src/hardware/mavlink/MavlinkEncoder.cpp
    src/hardware/mavlink/TelemetryStreamConfigurator.cpp
)
target_link_libraries(
    onboard_autonomy_hardware_mavlink
    PUBLIC
        onboard_autonomy_mission_types
        onboard_autonomy_mavlink_headers
        onboard_autonomy_public_headers
)
onboard_autonomy_enable_warnings(onboard_autonomy_hardware_mavlink)

add_library(
    onboard_autonomy_operator_screen
    src/operator/ui/screen/BoardTypeCatalog.cpp
)
target_link_libraries(
    onboard_autonomy_operator_screen
    PUBLIC
        onboard_autonomy_public_headers
)
onboard_autonomy_enable_warnings(onboard_autonomy_operator_screen)

add_library(
    onboard_autonomy_mission
    src/mission/autonomy/AutonomyRuntime.cpp
    src/mission/cv/AsyncCameraMonitor.cpp
    src/mission/cv/CameraMonitor.cpp
    src/mission/safety/CompanionLinkFailsafe.cpp
    src/mission/autonomy/DecisionEngine.cpp
    src/mission/flight/FlightStartupController.cpp
    src/mission/safety/MotionSafetyPolicy.cpp
    src/mission/safety/SafetySupervisor.cpp
    src/mission/cv/tracking/TargetTracker.cpp
    src/mission/cv/VisionMonitor.cpp
    src/mission/autonomy/WorldState.cpp
    src/mission/CompanionApplication.cpp
)
target_link_libraries(
    onboard_autonomy_mission
    PUBLIC
        onboard_autonomy_mission_types
        onboard_autonomy_mission_camera_port
        onboard_autonomy_mission_target_detector_port
        onboard_autonomy_public_headers
        onboard_autonomy_mission_transport_port
    PRIVATE
        onboard_autonomy_hardware_mavlink
)
onboard_autonomy_enable_warnings(onboard_autonomy_mission)

add_library(
    onboard_autonomy_mission_runtime
    src/bootstrap/MissionRuntime.cpp
)
target_link_libraries(
    onboard_autonomy_mission_runtime
    PUBLIC
        onboard_autonomy_mission
        onboard_autonomy_public_headers
    PRIVATE
        onboard_autonomy_mission_cv_detection
        onboard_autonomy_hardware_camera
        onboard_autonomy_mission_cv_geometry
        onboard_autonomy_hardware_transport
)
onboard_autonomy_enable_warnings(onboard_autonomy_mission_runtime)

add_library(
    onboard_autonomy_operator_ui
    src/operator/ui/input/ConsoleInput.cpp
    src/operator/ui/screen/ConsoleSnapshotSink.cpp
    src/operator/ui/screen/ConsoleView.cpp
)
target_link_libraries(
    onboard_autonomy_operator_ui
    PUBLIC
        onboard_autonomy_mission
        onboard_autonomy_public_headers
)
onboard_autonomy_enable_warnings(onboard_autonomy_operator_ui)

add_library(
    onboard_autonomy_diagnostics_logging
    src/diagnostics/logging/JsonDiagnosticSink.cpp
)
target_link_libraries(
    onboard_autonomy_diagnostics_logging
    PUBLIC
        onboard_autonomy_mission
        onboard_autonomy_public_headers
    PRIVATE
        nlohmann_json::nlohmann_json
)
onboard_autonomy_enable_warnings(onboard_autonomy_diagnostics_logging)

add_library(
    onboard_autonomy_operator_cli
    src/operator/cli/CommandLine.cpp
)
target_link_libraries(
    onboard_autonomy_operator_cli
    PUBLIC
        onboard_autonomy_public_headers
)
onboard_autonomy_enable_warnings(onboard_autonomy_operator_cli)

add_executable(
    onboard_autonomy
    src/main.cpp
    src/bootstrap/CompanionRunner.cpp
    src/bootstrap/Program.cpp
)
target_link_libraries(
    onboard_autonomy
    PRIVATE
        onboard_autonomy_operator_cli
        onboard_autonomy_operator_screen
        onboard_autonomy_diagnostics_preview
        onboard_autonomy_operator_ui
        onboard_autonomy_diagnostics_logging
        onboard_autonomy_mission_runtime
)
onboard_autonomy_enable_warnings(onboard_autonomy)

include(GNUInstallDirs)
install(
    TARGETS onboard_autonomy
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
    FILES third_party/ardupilot/board_types.txt
    DESTINATION ${CMAKE_INSTALL_DATADIR}/onboard_autonomy
    RENAME ardupilot-board-types.txt
)
install(
    FILES third_party/ardupilot/COPYING.txt
    DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/OnboardAutonomy
    RENAME ArduPilot-COPYING.txt
)
install(
    FILES ${apriltag_source_SOURCE_DIR}/LICENSE.md
    DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/OnboardAutonomy
    RENAME AprilTag-LICENSE.md
)
install(
    FILES ${cpp_httplib_source_SOURCE_DIR}/LICENSE
    DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/OnboardAutonomy
    RENAME cpp-httplib-LICENSE
)
install(
    FILES ${nlohmann_json_SOURCE_DIR}/LICENSE.MIT
    DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/OnboardAutonomy
    RENAME nlohmann-json-LICENSE.MIT
)
install(
    FILES assets/camera-preview/index.html
    DESTINATION ${CMAKE_INSTALL_DATADIR}/onboard_autonomy
    RENAME camera-preview.html
)
install(
    FILES assets/calibration/checkerboard-9x6-25mm-a4.svg
    DESTINATION ${CMAKE_INSTALL_DATADIR}/onboard_autonomy
)
install(
    FILES
        assets/apriltag/tagStandard41h12-id0-90mm-a4.svg
        assets/apriltag/tagStandard41h12-id0-90mm-a4-300dpi.png
    DESTINATION ${CMAKE_INSTALL_DATADIR}/onboard_autonomy
)
install(
    FILES
        config/camera-module-3-wide-640x480.json
        config/gazebo-landing-camera-640x480.json
        config/gazebo-landing-camera-extrinsics.json
    DESTINATION ${CMAKE_INSTALL_DATADIR}/onboard_autonomy
)

include(CTest)

if(BUILD_TESTING)
    add_executable(
        onboard_autonomy_tests
        tests/CompanionApplicationTests.cpp
        tests/JsonDiagnosticSinkTests.cpp
        tests/AutonomyCoreTests.cpp
        tests/AutonomyRuntimeTests.cpp
        tests/FlightStartupControllerTests.cpp
        tests/BoardTypeCatalogTests.cpp
        tests/CameraMonitorTests.cpp
        tests/CommandLineTests.cpp
        tests/CompanionLinkFailsafeTests.cpp
        tests/CameraCalibrationLoaderTests.cpp
        tests/CameraExtrinsicsLoaderTests.cpp
        tests/VisionMonitorTests.cpp
        tests/ConsoleViewTests.cpp
        tests/MavlinkDecoderTests.cpp
        tests/MavlinkEncoderTests.cpp
        tests/MotionSafetyPolicyTests.cpp
        tests/TelemetryStreamConfiguratorTests.cpp
        tests/TargetTrackerTests.cpp
        tests/TargetTransformTests.cpp
        tests/TransportTests.cpp
        tests/VehicleStateTests.cpp
        tests/YoloXImagePreprocessorTests.cpp
        tests/YoloXOutputDecoderTests.cpp
        tests/TestMain.cpp
    )
    target_link_libraries(
        onboard_autonomy_tests
        PRIVATE
            onboard_autonomy_mission
            onboard_autonomy_operator_cli
            onboard_autonomy_operator_screen
            onboard_autonomy_hardware_camera
            onboard_autonomy_mission_cv_geometry
            onboard_autonomy_mission_camera_port
            onboard_autonomy_diagnostics_preview_port
            onboard_autonomy_mission_cv_detection
            onboard_autonomy_mission_target_detector_port
            onboard_autonomy_mission_types
            onboard_autonomy_hardware_mavlink
            onboard_autonomy_operator_ui
            onboard_autonomy_diagnostics_logging
            onboard_autonomy_hardware_transport
            onboard_autonomy_mission_transport_port
            apriltag
            nlohmann_json::nlohmann_json
    )
    target_compile_definitions(
        onboard_autonomy_tests
        PRIVATE
            ONBOARD_AUTONOMY_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
    )
    target_include_directories(
        onboard_autonomy_tests
        SYSTEM PRIVATE
            ${apriltag_source_SOURCE_DIR}
    )
    onboard_autonomy_enable_warnings(onboard_autonomy_tests)
    add_test(NAME onboard_autonomy_tests COMMAND onboard_autonomy_tests)
endif()
