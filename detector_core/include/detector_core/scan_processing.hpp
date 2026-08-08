#pragma once

#include "detector_core/point2d.hpp"
#include <vector>

namespace detector_core {
    std::vector<Point2D> scan_to_points(
        const std::vector<double>& ranges,
        double angle_min,
        double angle_increment,
        double range_min,
        double range_max);
} // namespace detector_core