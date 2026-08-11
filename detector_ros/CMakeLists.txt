cmake_minimum_required(VERSION 3.8)

project(lidar_detector_ros)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_subdirectory(../detector_core detector_core_build EXCLUDE_FROM_ALL)

add_executable(lidar_detector_node src/lidar_detector_node.cpp)

add_executable(synthetic_scan_publisher src/synthetic_scan_publisher.cpp)

ament_target_dependencies(lidar_detector_node rclcpp sensor_msgs visualization_msgs)

ament_target_dependencies(synthetic_scan_publisher rclcpp sensor_msgs)

target_link_libraries(lidar_detector_node detector_core)

install(TARGETS lidar_detector_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS lidar_detector_node synthetic_scan_publisher DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()