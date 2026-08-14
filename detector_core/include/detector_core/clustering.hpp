#pragma once

#include "detector_core/point2d.hpp"
#include <cstddef>
#include <vector>
#include "detector_core/point3d.hpp"

namespace detector_core {
    using Cluster = std::vector<std::size_t>;

    std::vector<Cluster> euclidean_clusters(
        const std::vector<Point2D>& points,
        double distance_tolerance,
        std::size_t min_cluster_size,
        std::size_t max_cluster_size);
    
    std::vector<Cluster> ordered_euclidean_clusters(
        const std::vector<Point2D>& points, double distance_tolerance,
        std::size_t min_cluster_size, std::size_t max_cluster_size);

    std::vector<Cluster> spatial_grid_clusters(const std::vector<Point2D>& points,
        double distance_tolerance, std::size_t min_cluster_size, std::size_t max_cluster_size);

    std::vector<Cluster> spatial_grid_clusters_3d(
        const std::vector<Point3D>& points,
        double distance_tolerance,
        std::size_t min_cluster_size,
        std::size_t max_cluster_size);
} // namespace detector_core