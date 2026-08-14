#include "detector_core/point3d_processing.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

TEST(Point3DProcessingTest, RemovesInvalidAndOutOfRangePoints) {
    const std::vector<detector_core::Point3D> points{
        {1.0, 0.0, 0.0},
        {3.0, 0.0, 0.0},
        {10.0, 0.0, 0.0},
        {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}
    };

    const auto filtered =
        detector_core::filter_points(points, 0.5, 5.0);

    ASSERT_EQ(filtered.size(), 2);
    EXPECT_DOUBLE_EQ(filtered[0].x, 1.0);
    EXPECT_DOUBLE_EQ(filtered[1].x, 3.0);
}

TEST(Point3DProcessingTest, AcceptsPointsInsideRangeInThreeDimensions) {
    const std::vector<detector_core::Point3D> points{
        {1.0, 2.0, 2.0}
    };

    const auto filtered =
        detector_core::filter_points(points, 2.9, 3.1);

    ASSERT_EQ(filtered.size(), 1);
}

TEST(Point3DProcessingTest, MergesPointsInSameVoxel) {
    const std::vector<detector_core::Point3D> points{
        {0.01, 0.01, 0.01},
        {0.02, 0.02, 0.02},
        {1.01, 1.01, 1.01}
    };

    const auto downsampled =
        detector_core::voxel_downsample(points, 0.1);

    ASSERT_EQ(downsampled.size(), 2);
}

TEST(Point3DProcessingTest, ReturnsOriginalPointsForInvalidVoxelSize) {
    const std::vector<detector_core::Point3D> points{
        {1.0, 2.0, 3.0}
    };

    const auto downsampled =
        detector_core::voxel_downsample(points, 0.0);

    ASSERT_EQ(downsampled.size(), 1);
    EXPECT_DOUBLE_EQ(downsampled[0].x, 1.0);
}