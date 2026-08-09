#include "detector_core/clustering.hpp"
#include <vector>
#include <gtest/gtest.h>

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