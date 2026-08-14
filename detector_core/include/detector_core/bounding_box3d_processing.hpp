#pragma once

#include "detector_core/bounding_box3d.hpp"
#include "detector_core/point3d.hpp"
#include <cstddef>
#include <vector>

namespace detector_core {
    BoundingBox3D bounding_box3d_for_cluster(const std::vector<Point3D>& points, const std::vector<std::size_t>& cluster_indices);
} // namespace detector_core