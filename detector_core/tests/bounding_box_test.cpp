#include "detector_core/bounding_box.hpp"
#include <vector>
#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Dense>

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

TEST(OrientedBoundingBoxTest, StoresOrientationAndDimensions) {
    detector_core::OrientedBoundingBox2D box{
        2.0,   // center_x
        -1.0,  // center_y
        4.0,   // width
        1.5,   // height
        0.785398  // yaw, approximately 45 degrees
    };

    EXPECT_DOUBLE_EQ(box.center_x, 2.0);
    EXPECT_DOUBLE_EQ(box.center_y, -1.0);
    EXPECT_DOUBLE_EQ(box.width, 4.0);
    EXPECT_DOUBLE_EQ(box.height, 1.5);
    EXPECT_DOUBLE_EQ(box.yaw, 0.785398);
}

TEST(OrientedBoundingBoxTest, ComputesHorizontalBox) {
    const std::vector<detector_core::Point2D> points{
        {-2.0, -0.5},
        {2.0, -0.5},
        {2.0, 0.5},
        {-2.0, 0.5}};

    const std::vector<std::size_t> cluster_indices{0, 1, 2, 3};

    const detector_core::OrientedBoundingBox2D box =
        detector_core::oriented_bounding_box_for_cluster(
            points, cluster_indices);

    EXPECT_NEAR(box.center_x, 0.0, 1e-6);
    EXPECT_NEAR(box.center_y, 0.0, 1e-6);
    EXPECT_NEAR(box.width, 4.0, 1e-6);
    EXPECT_NEAR(box.height, 1.0, 1e-6);
    EXPECT_NEAR(box.yaw, 0.0, 1e-6);
}

TEST(OrientedBoundingBoxTest, ComputesFortyFiveDegreeBox) {
    const double angle = 3.14159265358979323846 / 4.0;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);

    const double center_x = 1.0;
    const double center_y = -2.0;

    const double half_width = 2.0;
    const double half_height = 0.5;

    const Eigen::Vector2d major_axis(cosine, sine);
    const Eigen::Vector2d minor_axis(-sine, cosine);

    const Eigen::Vector2d center(center_x, center_y);

    const std::vector<detector_core::Point2D> points{
        {
            (center + half_width * major_axis + half_height * minor_axis).x(),
            (center + half_width * major_axis + half_height * minor_axis).y()
        },
        {
            (center + half_width * major_axis - half_height * minor_axis).x(),
            (center + half_width * major_axis - half_height * minor_axis).y()
        },
        {
            (center - half_width * major_axis + half_height * minor_axis).x(),
            (center - half_width * major_axis + half_height * minor_axis).y()
        },
        {
            (center - half_width * major_axis - half_height * minor_axis).x(),
            (center - half_width * major_axis - half_height * minor_axis).y()
        }
    };

    const std::vector<std::size_t> cluster_indices{0, 1, 2, 3};

    const detector_core::OrientedBoundingBox2D box =
        detector_core::oriented_bounding_box_for_cluster(
            points, cluster_indices);

    EXPECT_NEAR(box.center_x, center_x, 1e-6);
    EXPECT_NEAR(box.center_y, center_y, 1e-6);
    EXPECT_NEAR(box.width, 4.0, 1e-6);
    EXPECT_NEAR(box.height, 1.0, 1e-6);
    EXPECT_NEAR(box.yaw, angle, 1e-6);
}