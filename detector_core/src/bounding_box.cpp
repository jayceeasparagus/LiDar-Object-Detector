#include "detector_core/bounding_box.hpp"

namespace detector_core {
    BoundingBox2D bounding_box_for_cluster(const std::vector<Point2D>& points, const std::vector<std::size_t>& cluster_indices) {
        const Point2D& first_point = points[cluster_indices[0]];

        BoundingBox2D box {first_point.x, first_point.y, first_point.x, first_point.y};

        for (const std::size_t i : cluster_indices) {
            const Point2D& point = points[i];

            if (point.x < box.min_x) {
                box.min_x = point.x;
            }
            if (point.y < box.min_y) {
                box.min_y = point.y;
            }
            if (point.x < box.min_x) {
                box.max_x = point.x;
            }
            if (point.y < box.min_y) {
                box.max_y = point.y;
            }
        }

        return box;
    }
} // namespace detector_core