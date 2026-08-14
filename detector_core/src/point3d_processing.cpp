#include "detector_core/point3d_processing.hpp"
#include <cmath>
#include <cstddef>
#include <unordered_map>

namespace detector_core {
    struct VoxelKey {
        int x;
        int y;
        int z;

        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    std::vector<Point3D> filter_points(const std::vector<Point3D>& points, double min_range, double max_range) {
        std::vector<Point3D> filtered_points;
        filtered_points.reserve(points.size());

        for (const Point3D& point : points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }

            const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

            if (range >= min_range && range <= max_range) {
                filtered_points.push_back(point);
            }
        }

        return filtered_points;
    }

    struct VoxelKeyHash {
        std::size_t operator()(const VoxelKey& key) const {
            const std::size_t hx = std::hash<int>{}(key.x);
            const std::size_t hy = std::hash<int>{}(key.y);
            const std::size_t hz = std::hash<int>{}(key.z);

            return hx ^ (hy << 1) ^ (hz << 2);
        }
    };

    struct VoxelAccumulator {
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        std::size_t count = 0;
    };

    std::vector<Point3D> voxel_downsample(
        const std::vector<Point3D>& points,
        double voxel_size) {

        if (voxel_size <= 0.0) {
            return points;
        }

        std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash>
            accumulators;
        accumulators.reserve(points.size());

        for (const Point3D& point : points) {
            const VoxelKey key{
                static_cast<int>(std::floor(point.x / voxel_size)),
                static_cast<int>(std::floor(point.y / voxel_size)),
                static_cast<int>(std::floor(point.z / voxel_size))};

            VoxelAccumulator& accumulator = accumulators[key];
            accumulator.sum_x += point.x;
            accumulator.sum_y += point.y;
            accumulator.sum_z += point.z;
            ++accumulator.count;
        }

        std::vector<Point3D> downsampled_points;
        downsampled_points.reserve(accumulators.size());

        for (const auto& entry : accumulators) {
            const VoxelAccumulator& accumulator = entry.second;
            downsampled_points.push_back(Point3D{
                accumulator.sum_x / static_cast<double>(accumulator.count),
                accumulator.sum_y / static_cast<double>(accumulator.count),
                accumulator.sum_z / static_cast<double>(accumulator.count)});
        }

        return downsampled_points;
    }
} // namespace detector_core