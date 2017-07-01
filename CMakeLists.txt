cmake_minimum_required (VERSION 3.7)
project (robot-pathfinder)

# The version number.
set (robot-pathfinder_VERSION_MAJOR 0)
set (robot-pathfinder_VERSION_MINOR 1)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Qt5Core)
find_package(Qt5Widgets)

add_executable(robot-pathfinder robot-pathfinder.cpp robot-mapwidget.cpp robot-map.cpp robot-geometry.cpp)

#target_compile_features(robot-pathfinder PRIVATE cxx_auto_type)

# Use the Widgets module from Qt 5.
target_link_libraries(robot-pathfinder Qt5::Widgets Qt5::Core)
