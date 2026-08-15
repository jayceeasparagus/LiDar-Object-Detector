#include "detector_core/clustering.hpp"
#include "detector_core/point3d_processing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <queue>
#include <vector>

namespace {

template <typename Function>
double average_runtime_ms(Function function, int repetitions) {
    for (int i = 0; i < 2; ++i) {
        function();
    }

    volatile std::size_t result_sink = 0;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < repetitions; ++i) {
        const auto result = function();
        result_sink += result.size();
    }
    const auto end = std::chrono::steady_clock::now();
    (void)result_sink;

    return std::chrono::duration<double, std::milli>(end - start).count() /
        static_cast<double>(repetitions);
}

std::vector<detector_core::Cluster> brute_force_clusters_3d(
    const std::vector<detector_core::Point3D>& points,
    double tolerance,
    std::size_t min_cluster_size,
    std::size_t max_cluster_size) {
    std::vector<detector_core::Cluster> clusters;
    std::vector<bool> visited(points.size(), false);
    const double tolerance_squared = tolerance * tolerance;

    for (std::size_t seed = 0; seed < points.size(); ++seed) {
        if (visited[seed]) continue;

        detector_core::Cluster cluster;
        std::queue<std::size_t> pending;
        visited[seed] = true;
        pending.push(seed);

        while (!pending.empty()) {
            const std::size_t current = pending.front();
            pending.pop();
            cluster.push_back(current);

            for (std::size_t candidate = 0; candidate < points.size(); ++candidate) {
                if (visited[candidate]) continue;
                const double dx = points[current].x - points[candidate].x;
                const double dy = points[current].y - points[candidate].y;
                const double dz = points[current].z - points[candidate].z;
                if (dx * dx + dy * dy + dz * dz <= tolerance_squared) {
                    visited[candidate] = true;
                    pending.push(candidate);
                }
            }
        }

        if (cluster.size() >= min_cluster_size && cluster.size() <= max_cluster_size) {
            clusters.push_back(std::move(cluster));
        }
    }

    return clusters;
}

std::vector<detector_core::Point3D> make_points(std::size_t count) {
    constexpr std::size_t cluster_count = 10;
    constexpr double spacing = 0.025;
    constexpr std::size_t side = 12;

    std::vector<detector_core::Point3D> points;
    points.reserve(count);

    for (std::size_t index = 0; index < count; ++index) {
        const std::size_t cluster = index % cluster_count;
        const std::size_t local = index / cluster_count;
        const std::size_t x_index = local % side;
        const std::size_t y_index = (local / side) % side;
        const std::size_t z_index = (local / (side * side)) % side;

        points.push_back({
            static_cast<double>(cluster) * 3.0 + static_cast<double>(x_index) * spacing,
            static_cast<double>(y_index) * spacing,
            0.5 + static_cast<double>(z_index) * spacing});
    }

    return points;
}

std::vector<detector_core::Cluster> canonicalize(
    std::vector<detector_core::Cluster> clusters) {
    for (auto& cluster : clusters) std::sort(cluster.begin(), cluster.end());
    std::sort(clusters.begin(), clusters.end());
    return clusters;
}

int repetitions_for(std::size_t point_count) {
    if (point_count <= 1024) return 50;
    if (point_count <= 3100) return 15;
    if (point_count <= 5000) return 6;
    return 2;
}

} // namespace

int main() {
    constexpr double tolerance = 0.12;
    constexpr double voxel_size = 0.05;
    constexpr std::size_t min_cluster_size = 2;
    constexpr std::size_t max_cluster_size = 100000;
    const std::vector<std::size_t> sizes{1024, 3100, 5000, 10240};

    std::cout << "3D clustering benchmark (same input, same result)\n";
    std::cout << "points,clusters,brute_force_ms,spatial_grid_ms,speedup,correct\n";
    std::cout << std::fixed << std::setprecision(4);

    for (const std::size_t size : sizes) {
        const auto points = make_points(size);
        const auto brute_result = brute_force_clusters_3d(
            points, tolerance, min_cluster_size, max_cluster_size);
        const auto grid_result = detector_core::spatial_grid_clusters_3d(
            points, tolerance, min_cluster_size, max_cluster_size);
        const bool correct = canonicalize(brute_result) == canonicalize(grid_result);
        const int repetitions = repetitions_for(size);

        const double brute_ms = average_runtime_ms([&]() {
            return brute_force_clusters_3d(
                points, tolerance, min_cluster_size, max_cluster_size);
        }, repetitions);
        const double grid_ms = average_runtime_ms([&]() {
            return detector_core::spatial_grid_clusters_3d(
                points, tolerance, min_cluster_size, max_cluster_size);
        }, repetitions);

        std::cout << size << ',' << brute_result.size() << ','
                  << brute_ms << ',' << grid_ms << ','
                  << (brute_ms / grid_ms) << ','
                  << (correct ? "yes" : "NO") << '\n';
        if (!correct) return 1;
    }

    std::cout << "\n3D pipeline benchmark (raw brute force vs voxel + grid)\n";
    std::cout << "raw_points,voxel_points,reduction_percent,brute_raw_ms,voxel_ms,grid_after_voxel_ms,optimized_total_ms,pipeline_speedup,cluster_count_match\n";

    for (const std::size_t size : sizes) {
        const auto raw_points = make_points(size);
        const auto voxel_points = detector_core::voxel_downsample(raw_points, voxel_size);
        const auto brute_result = brute_force_clusters_3d(
            raw_points, tolerance, min_cluster_size, max_cluster_size);
        const auto optimized_result = detector_core::spatial_grid_clusters_3d(
            voxel_points, tolerance, min_cluster_size, max_cluster_size);
        const int repetitions = repetitions_for(size);

        const double brute_ms = average_runtime_ms([&]() {
            return brute_force_clusters_3d(
                raw_points, tolerance, min_cluster_size, max_cluster_size);
        }, repetitions);
        const double voxel_ms = average_runtime_ms([&]() {
            return detector_core::voxel_downsample(raw_points, voxel_size);
        }, repetitions);
        const double grid_ms = average_runtime_ms([&]() {
            return detector_core::spatial_grid_clusters_3d(
                voxel_points, tolerance, min_cluster_size, max_cluster_size);
        }, repetitions);
        const double pipeline_ms = average_runtime_ms([&]() {
            const auto downsampled = detector_core::voxel_downsample(raw_points, voxel_size);
            return detector_core::spatial_grid_clusters_3d(
                downsampled, tolerance, min_cluster_size, max_cluster_size);
        }, repetitions);

        const double reduction = 100.0 *
            (1.0 - static_cast<double>(voxel_points.size()) / static_cast<double>(raw_points.size()));

        std::cout << size << ',' << voxel_points.size() << ',' << reduction << ','
                  << brute_ms << ',' << voxel_ms << ',' << grid_ms << ','
                  << pipeline_ms << ',' << (brute_ms / pipeline_ms) << ','
                  << (brute_result.size() == optimized_result.size() ? "yes" : "NO") << '\n';
    }

    return 0;
}
