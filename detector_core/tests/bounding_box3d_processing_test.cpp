#include "detector_core/bounding_box3d_processing.hpp"

#include <gtest/gtest.h>

TEST(BoundingBox3DProcessingTest, ComputesAxisAlignedBox) {
    const std::vector<detector_core::Point3D> points{
        {-2.0, -1.0, 0.0},
        {2.0, -1.0, 0.0},
        {2.0, 1.0, 2.0},
        {-2.0, 1.0, 2.0}
    };

    const std::vector<std::size_t> cluster_indices{
        0, 1, 2, 3
    };

    const detector_core::BoundingBox3D box =
        detector_core::bounding_box3d_for_cluster(
            points, cluster_indices);

    EXPECT_NEAR(box.center_x, 0.0, 1e-6);
    EXPECT_NEAR(box.center_y, 0.0, 1e-6);
    EXPECT_NEAR(box.center_z, 1.0, 1e-6);

    EXPECT_NEAR(box.width, 4.0, 1e-6);
    EXPECT_NEAR(box.depth, 2.0, 1e-6);
    EXPECT_NEAR(box.height, 2.0, 1e-6);

    EXPECT_NEAR(box.yaw, 0.0, 1e-6);
}