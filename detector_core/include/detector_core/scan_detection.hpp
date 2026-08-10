#pragma once

#include "detector_core/bounding_box.hpp"
#include <cstddef>
#include <vector>

namespace detector_core {
    std::vector<BoundingBox2D> detect_obstacles_from_scan(const std::vector<double>& ranges,
    double angle_min, double angle_increment, double range_min, double range_max,
    double distance_tolerance, std::size_t min_cluster_size, std::size_t max_cluster_size);
} // namespace detector_core