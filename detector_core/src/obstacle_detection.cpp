#include "detector_core/obstacle_detection.hpp"
#include "detector_core/clustering.hpp"

namespace detector_core {
    DetectionResult detect_obstacles_with_points(const std::vector<Point2D>& points, 
    double distance_tolerance, std::size_t min_cluster_size, std::size_t max_cluster_size) {
        const std::vector<Cluster> clusters = euclidean_clusters(points, distance_tolerance, min_cluster_size, max_cluster_size);

        DetectionResult result;
        result.bounding_boxes.reserve(clusters.size());

        for (const Cluster& cluster : clusters) {
            result.bounding_boxes.push_back(bounding_box_for_cluster(points, cluster));

            for (const std::size_t i : cluster) {
                result.obstacle_points.push_back(points[i]);
            }
        }

        return result;
    }

    std::vector<BoundingBox2D> detect_obstacles(const std::vector<Point2D>& points, double distance_tolerance,
    std::size_t min_cluster_size, std::size_t max_cluster_size) {
        const std::vector<Cluster> clusters = euclidean_clusters(points, distance_tolerance, min_cluster_size, max_cluster_size);

        std::vector<BoundingBox2D> boxes;
        boxes.reserve(clusters.size());

        for (const Cluster& cluster : clusters) {
            boxes.push_back(bounding_box_for_cluster(points, cluster));
        }

        return boxes;
    }
} // namespace detector_core 