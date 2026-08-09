#include "detector_core/bounding_box.hpp"
#include <vector>
#include <gtest/gtest.h>

TEST(BoundingBoxTest, ComputesMinimumAndMaximumCoordinates) {
    const std::vector<detector_core::Point2D> points{
        {-1.0, 2.0}, {3.0, -4.0}, {0.0, 5.0}, {100.0, 100.0}};

    const std::vector<std::size_t> cluster_indices{0, 1, 2};

    const detector_core::BoundingBox2D box = detector_core::bounding_box_for_cluster(points, cluster_indices);

    EXPECT_DOUBLE_EQ(box.min_x, -1.0);
    EXPECT_DOUBLE_EQ(box.min_y, -4.0);
    EXPECT_DOUBLE_EQ(box.max_x, 3.0);
    EXPECT_DOUBLE_EQ(box.max_y, 5.0);
}