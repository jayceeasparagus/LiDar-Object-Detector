#include "detector_core/scan_detection.hpp"
#include "detector_core/obstacle_detection.hpp"
#include "detector_core/scan_processing.hpp"

namespace detector_core {
    std::vector<BoundingBox2D> detect_obstacles_from_scan(const std::vector<double>& ranges, double angle_min,
    double angle_increment, double range_min, double range_max, double distance_tolerance,
    std::size_t min_cluster_size, std::size_t max_cluster_size) {
        const std::vector<Point2D> points = scan_to_points(ranges, angle_min, angle_increment, range_min, range_max);

        return detect_obstacles(points, distance_tolerance, min_cluster_size, max_cluster_size);
    }
} // namespace detector_core