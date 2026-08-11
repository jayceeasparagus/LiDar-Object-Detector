cmake_minimum_required(VERSION 3.20)

project(lidar_object_detector LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
enable_testing()

add_subdirectory(detector_core)
add_subdirectory(benchmarks)