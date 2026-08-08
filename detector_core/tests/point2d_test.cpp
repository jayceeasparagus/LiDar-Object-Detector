#include "detector_core/point2d.hpp"
#include <gtest/gtest.h>

TEST(Point2DTest, StoresCoordinates) {
    detector_core::Point2D point{1.5, -2.0};

    EXPECT_DOUBLE_EQ(point.x, 1.5);
    EXPECT_DOUBLE_EQ(point.y, -2.0);
}