find_package(Eigen3 REQUIRED)

add_library(detector_core STATIC src/library.cpp src/polar_conversion.cpp src/scan_processing.cpp
src/clustering.cpp src/bounding_box.cpp src/obstacle_detection.cpp src/scan_detection.cpp
src/kalman_filter_2d.cpp)

target_include_directories(detector_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_features(detector_core PUBLIC cxx_std_17)

target_link_libraries(detector_core PUBLIC Eigen3::Eigen)

find_package(GTest REQUIRED)

add_executable(detector_core_tests tests/point2d_test.cpp tests/polar_conversion_test.cpp tests/scan_processing_test.cpp
tests/clustering_test.cpp tests/bounding_box_test.cpp tests/obstacle_detection_test.cpp tests/scan_detection_test.cpp
tests/detection_result_test.cpp tests/kalman_filter_2d_test.cpp)

target_link_libraries(detector_core_tests PRIVATE detector_core GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(detector_core_tests)