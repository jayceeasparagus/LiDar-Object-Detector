#pragma once

#include "detector_core/point2d.hpp"
#include <cstddef>
#include <vector>

namespace detector_core {
    struct BoundingBox2D {
        double min_x;
        double min_y;
        double max_x;
        double max_y;
    };

    struct OrientedBoundingBox2D {
        double center_x;
        double center_y;
        double width;
        double height;
        double yaw;
    };

    BoundingBox2D bounding_box_for_cluster(const std::vector<Point2D>& points, const std::vector<std::size_t>& cluster_indices);

    OrientedBoundingBox2D oriented_bounding_box_for_cluster(const std::vector<Point2D>& points, const std::vector<std::size_t>& cluster_indices);
} // namespace detector_core