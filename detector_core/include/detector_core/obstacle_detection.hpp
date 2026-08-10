#pragma once

#include "detector_core/bounding_box.hpp"
#include <cstddef>
#include <vector>
#include "detector_core/detection_result.hpp"

namespace detector_core {
    DetectionResult detect_obstacles_with_points(const std::vector<Point2D>& points,
    double distance_tolerance, std::size_t min_cluster_size, std::size_t max_cluster_size);
    std::vector<BoundingBox2D> detect_obstacles(const std::vector<Point2D>& points, double distance_tolerance,
    std::size_t min_cluster_size, std::size_t max_cluster_size);
} // namespace detector_core