#include "detector_core/obstacle_detection.hpp"
#include <vector>
#include <gtest/gtest.h>

TEST(ObstacleDetectionTest, ProducesBoundingBoxesForPointClusters) {
    const std::vector<detector_core::Point2D> points {
        {0.0, 0.0}, {0.1, 0.0}, {5.0, 5.0}, {5.1, 5.0}, {10.0, 10.0}};

    const std::vector<detector_core::BoundingBox2D> boxes = detector_core::detect_obstacles(
        points, 0.2, 2, 3);

    ASSERT_EQ(boxes.size(), 2);

    EXPECT_NEAR(boxes[0].min_x, 0.0, 1e-9);
    EXPECT_NEAR(boxes[0].min_y, 0.0, 1e-9);
    EXPECT_NEAR(boxes[0].max_x, 0.1, 1e-9);

    EXPECT_NEAR(boxes[1].min_x, 5.0, 1e-9);
    EXPECT_NEAR(boxes[1].min_y, 5.0, 1e-9);
    EXPECT_NEAR(boxes[1].max_x, 5.1, 1e-9);
    EXPECT_NEAR(boxes[1].max_y, 5.0, 1e-9);
}