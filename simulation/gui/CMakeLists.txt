cmake_minimum_required(VERSION 3.22)

project(OnboardAutonomyGazeboGui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)

if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
endif()

find_package(Qt5 REQUIRED COMPONENTS Core Qml Quick QuickControls2)
find_package(gz-gui8 REQUIRED)

qt5_add_resources(wind_indicator_resources wind_indicator.qrc)

add_library(
    WindIndicator
    SHARED
    WindIndicator.cc
    WindIndicator.hh
    ${wind_indicator_resources}
)

target_link_libraries(
    WindIndicator
    PRIVATE
        gz-gui8::gz-gui8
        Qt5::Core
        Qt5::Qml
        Qt5::Quick
        Qt5::QuickControls2
)

target_compile_options(
    WindIndicator
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
)
