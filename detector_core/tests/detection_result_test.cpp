#include "detector_core/obstacle_detection.hpp"
#include <vector>
#include <gtest/gtest.h>

TEST(DetectionResultTest, ReturnsObstaclePointsAndBoundingBoxes) {
    const std::vector<detector_core::Point2D> points{
        {0.0, 0.0}, {0.1, 0.0}, {5.0, 5.0}, {5.1, 5.0}, {10.0, 10.0}};

    const detector_core::DetectionResult result = detector_core::detect_obstacles_with_points(points, 0.2, 2, 3);

    ASSERT_EQ(result.bounding_boxes.size(), 2);
    ASSERT_EQ(result.oriented_bounding_boxes.size(), 2);
    ASSERT_EQ(result.obstacle_points.size(), 4);

    EXPECT_DOUBLE_EQ(result.obstacle_points[0].x, 0.0);
    EXPECT_DOUBLE_EQ(result.obstacle_points[0].y, 0.0);

    EXPECT_DOUBLE_EQ(result.obstacle_points[1].x, 0.1);
    EXPECT_DOUBLE_EQ(result.obstacle_points[1].y, 0.0);

    EXPECT_DOUBLE_EQ(result.obstacle_points[2].x, 5.0);
    EXPECT_DOUBLE_EQ(result.obstacle_points[2].y, 5.0);

    EXPECT_DOUBLE_EQ(result.obstacle_points[3].x, 5.1);
    EXPECT_DOUBLE_EQ(result.obstacle_points[3].y, 5.0);
}