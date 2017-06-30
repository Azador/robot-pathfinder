cmake_minimum_required (VERSION 3.7)
project (robot-pathfinder)

# The version number.
set (robot-pathfinder_VERSION_MAJOR 0)
set (robot-pathfinder_VERSION_MINOR 1)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Qt5Widgets)

add_executable(robot-pathfinder robot-pathfinder.cpp robot-mapwidget.cpp robot-map.cpp robot-geometry.cpp)

target_compile_features(robot-pathfinder PRIVATE cxx_auto_type)
qt5_use_modules(robot-pathfinder Core Gui Widgets)

