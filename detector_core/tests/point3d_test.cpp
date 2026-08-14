#include "detector_core/point3d.hpp"
#include <gtest/gtest.h>

TEST(Point3DTest, StoresCoordinates) {
    const detector_core::Point3D point{1.0, -2.0, 3.5};

    EXPECT_DOUBLE_EQ(point.x, 1.0);
    EXPECT_DOUBLE_EQ(point.y, -2.0);
    EXPECT_DOUBLE_EQ(point.z, 3.5);
}