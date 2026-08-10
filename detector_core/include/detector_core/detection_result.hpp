#pragma once

#include "detector_core/bounding_box.hpp"
#include "detector_core/point2d.hpp"
#include <vector>

namespace detector_core {
    struct DetectionResult {
        std::vector<Point2D> obstacle_points;
        std::vector<BoundingBox2D> bounding_boxes;
    };
} // namespace detector_core