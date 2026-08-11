#include "detector_core/clustering.hpp"
#include <vector>
#include <gtest/gtest.h>
#include <algorithm>
#include <random>

TEST(ClusteringTest, GroupsNearbyPointsAndRejectsSmallClusters) {
    const std::vector<detector_core::Point2D> points{
        {0.0, 0.0}, {0.1, 0.0}, {5.0, 5.0}, {5.1, 5.0}, {10.0, 10.0}
    };

    const std::vector<detector_core::Cluster> clusters = detector_core::euclidean_clusters(points, 0.2, 2, 3);

    ASSERT_EQ(clusters.size(), 2);

    ASSERT_EQ(clusters[0].size(), 2);
    EXPECT_EQ(clusters[0][0], 0);
    EXPECT_EQ(clusters[0][1], 1);

    ASSERT_EQ(clusters[1].size(), 2);
    EXPECT_EQ(clusters[1][0], 2);
    EXPECT_EQ(clusters[1][1], 3);
}

TEST(ClusteringTest, OrderedAlgorithmGroupsContiguousPoints) {
    const std::vector<detector_core::Point2D> points {
        {0.0, 0.0}, {0.1, 0.0}, {5.0, 5.0}, {5.1, 5.0}, {10.0, 10.0}};

    const std::vector<detector_core::Cluster> clusters = detector_core::ordered_euclidean_clusters(points, 0.2, 2, 3);

    ASSERT_EQ(clusters.size(), 2);
    
    ASSERT_EQ(clusters[0].size(), 2);
    EXPECT_EQ(clusters[0][0], 0);
    EXPECT_EQ(clusters[0][1], 1);

    ASSERT_EQ(clusters[1].size(), 2);
    EXPECT_EQ(clusters[1][0], 2);
    EXPECT_EQ(clusters[1][1], 3);
}

TEST(ClusteringTest, SeparatesInterleavedObstacles)
{
    const std::vector<detector_core::Point2D> points{
        {0.0, 0.0}, {5.0, 0.0}, {0.1, 0.0}, {5.1, 0.0}};

    const auto clusters = detector_core::euclidean_clusters(points, 0.2, 2, 10);

    ASSERT_EQ(clusters.size(), 2);
    EXPECT_EQ(clusters[0].size(), 2);
    EXPECT_EQ(clusters[1].size(), 2);
}

TEST(ClusteringTest, SpatialGridMatchesBruteForce)
{
    const std::vector<detector_core::Point2D> points{
        {0.0, 0.0},
        {5.0, 0.0},
        {0.1, 0.0},
        {5.1, 0.0},

        {-2.0, -2.0},
        {-1.9, -2.0},

        {10.0, 0.0},
        {10.15, 0.0},
        {10.3, 0.0}
    };

    constexpr double tolerance = 0.2;
    constexpr std::size_t minimum_size = 2;
    constexpr std::size_t maximum_size = 10;

    const auto brute_force_clusters =
        detector_core::euclidean_clusters(
            points,
            tolerance,
            minimum_size,
            maximum_size);

    const auto grid_clusters =
        detector_core::spatial_grid_clusters(
            points,
            tolerance,
            minimum_size,
            maximum_size);

    ASSERT_EQ(
        brute_force_clusters.size(),
        grid_clusters.size());

    std::vector<std::size_t> brute_force_sizes;
    std::vector<std::size_t> grid_sizes;

    for (const auto& cluster : brute_force_clusters) {
        brute_force_sizes.push_back(cluster.size());
    }

    for (const auto& cluster : grid_clusters) {
        grid_sizes.push_back(cluster.size());
    }

    std::sort(brute_force_sizes.begin(), brute_force_sizes.end());
    std::sort(grid_sizes.begin(), grid_sizes.end());

    EXPECT_EQ(brute_force_sizes, grid_sizes);
}

TEST(ClusteringTest, SpatialGridHandlesEmptyInput)
{
    const std::vector<detector_core::Point2D> points;

    const auto clusters =
        detector_core::spatial_grid_clusters(points, 0.2, 2, 10);

    EXPECT_TRUE(clusters.empty());
}

TEST(ClusteringTest, SpatialGridGroupsDuplicatePoints)
{
    const std::vector<detector_core::Point2D> points{
        {1.0, 1.0},
        {1.0, 1.0},
        {1.0, 1.0}
    };

    const auto clusters =
        detector_core::spatial_grid_clusters(points, 0.2, 2, 10);

    ASSERT_EQ(clusters.size(), 1);
    EXPECT_EQ(clusters[0].size(), 3);
}

TEST(ClusteringTest, SpatialGridHandlesCellBoundaries)
{
    const std::vector<detector_core::Point2D> points{
        {0.49, 0.0},
        {0.51, 0.0},
        {-0.49, 0.0},
        {-0.51, 0.0}
    };

    const auto clusters =
        detector_core::spatial_grid_clusters(points, 0.5, 2, 10);

    ASSERT_EQ(clusters.size(), 2);
    EXPECT_EQ(clusters[0].size(), 2);
    EXPECT_EQ(clusters[1].size(), 2);
}

TEST(ClusteringTest, SpatialGridPreservesTransitiveConnectivity)
{
    const std::vector<detector_core::Point2D> points{
        {0.0, 0.0},
        {0.15, 0.0},
        {0.30, 0.0}
    };

    const auto clusters =
        detector_core::spatial_grid_clusters(points, 0.2, 2, 10);

    ASSERT_EQ(clusters.size(), 1);
    EXPECT_EQ(clusters[0].size(), 3);
}

TEST(ClusteringTest, SpatialGridMatchesBruteForceOnGeneratedData)
{
    std::mt19937 generator(42);
    std::normal_distribution<double> noise(0.0, 0.05);

    const std::vector<detector_core::Point2D> centers{
        {0.0, 0.0},
        {3.0, 0.0},
        {6.0, 0.0},
        {0.0, 3.0},
        {3.0, 3.0},
        {6.0, 3.0},
        {0.0, 6.0},
        {3.0, 6.0}
    };

    std::vector<detector_core::Point2D> points;

    for (const auto& center : centers) {
        for (int point_number = 0; point_number < 25; ++point_number) {
            points.push_back({
                center.x + noise(generator),
                center.y + noise(generator)
            });
        }
    }

    constexpr double tolerance = 0.2;
    constexpr std::size_t minimum_size = 2;
    constexpr std::size_t maximum_size = 100;

    const auto brute_force_clusters =
        detector_core::euclidean_clusters(
            points,
            tolerance,
            minimum_size,
            maximum_size);

    const auto grid_clusters =
        detector_core::spatial_grid_clusters(
            points,
            tolerance,
            minimum_size,
            maximum_size);

    ASSERT_EQ(
        brute_force_clusters.size(),
        grid_clusters.size());

    std::vector<std::size_t> brute_force_sizes;
    std::vector<std::size_t> grid_sizes;

    for (const auto& cluster : brute_force_clusters) {
        brute_force_sizes.push_back(cluster.size());
    }

    for (const auto& cluster : grid_clusters) {
        grid_sizes.push_back(cluster.size());
    }

    std::sort(brute_force_sizes.begin(), brute_force_sizes.end());
    std::sort(grid_sizes.begin(), grid_sizes.end());

    EXPECT_EQ(brute_force_sizes, grid_sizes);
}