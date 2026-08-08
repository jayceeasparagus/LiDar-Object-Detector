#include "detector_core/scan_processing.hpp"
#include <cmath>
#include <limits>
#include <vector>
#include <gtest/gtest.h>

TEST(ScanProcessingTest, ConvertsValidRangesAndSkipsInvalidRanges) {
    constexpr double pi = 3.14159265358979323846;

    const std::vector<double> ranges{
        1.0,
        std::numeric_limits<double>::quiet_NaN(),
        3.0,
        std::numeric_limits<double>::infinity(),
        0.5,
        4.0
    };

    const std::vector<detector_core::Point2D> points = detector_core::scan_to_points(
        ranges,
        0.0,
        pi / 2.0,
        1.0,
        3.0);

    ASSERT_EQ(points.size(), 2);

    EXPECT_NEAR(points[0].x, 1.0, 1e-9);
    EXPECT_NEAR(points[0].y, 0.0, 1e-9);

    EXPECT_NEAR(points[1].x, -3.0, 1e-9);
    EXPECT_NEAR(points[1].y, 0.0, 1e-9);
}