#pragma once

#include "detector_core/point3d.hpp"
#include <vector>

namespace detector_core {
    std::vector<Point3D> filter_points(const std::vector<Point3D>& points, double min_range, double max_range);

    std::vector<Point3D> voxel_downsample(const std::vector<Point3D>& points, double voxel_size);
} // namespace detector_core