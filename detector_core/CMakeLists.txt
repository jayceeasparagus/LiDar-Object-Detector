add_library(detector_core STATIC src/library.cpp src/polar_conversion.cpp src/scan_processing.cpp
src/clustering.cpp src/bounding_box.cpp)

target_include_directories(detector_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_features(detector_core PUBLIC cxx_std_17)

find_package(GTest REQUIRED)

add_executable(detector_core_tests tests/point2d_test.cpp tests/polar_conversion_test.cpp tests/scan_processing_test.cpp
tests/clustering_test.cpp tests/bounding_box_test.cpp)

target_link_libraries(detector_core_tests PRIVATE detector_core GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(detector_core_tests)