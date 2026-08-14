#include "detector_core/bounding_box3d.hpp"
#include <gtest/gtest.h>

TEST(BoundingBox3DTest, StoresDimensionsAndYaw) {
    const detector_core::BoundingBox3D box{
        1.0,   // center_x
        -2.0,  // center_y
        0.75,  // center_z
        4.0,   // width
        2.0,   // depth
        1.5,   // height
        0.785398  // yaw
    };

    EXPECT_DOUBLE_EQ(box.center_x, 1.0);
    EXPECT_DOUBLE_EQ(box.center_y, -2.0);
    EXPECT_DOUBLE_EQ(box.center_z, 0.75);
    EXPECT_DOUBLE_EQ(box.width, 4.0);
    EXPECT_DOUBLE_EQ(box.depth, 2.0);
    EXPECT_DOUBLE_EQ(box.height, 1.5);
    EXPECT_DOUBLE_EQ(box.yaw, 0.785398);
}