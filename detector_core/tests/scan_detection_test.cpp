#include "detector_core/scan_detection.hpp"
#include <cmath>
#include <limits>
#include <vector>
#include <gtest/gtest.h>

TEST(ScanDetectionTest, ConvertsScanIntoObstacleBoxes) {
    const std::vector<double> ranges{1.0, 1.05, std::numeric_limits<double>::quiet_NaN(), 5.0, 5.05};

    const std::vector<detector_core::BoundingBox2D> boxes = detector_core::detect_obstacles_from_scan(
        ranges, 0.0, 0.01, 0.1, 10.0, 0.2, 2, 3);

        ASSERT_EQ(boxes.size(), 2);

        EXPECT_NEAR(boxes[0].min_x, 1.0, 1e-9);
        EXPECT_NEAR(boxes[0].max_x, 1.05, 0.01);

        EXPECT_NEAR(boxes[1].min_x, 5.0, 0.01);
        EXPECT_NEAR(boxes[1].max_x, 5.05, 0.01);
}