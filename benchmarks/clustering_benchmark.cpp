#include "detector_core/clustering.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

template <typename Function>
double average_runtime_ms(Function function, int repetitions) {
    for (int i = 0; i < 3; ++i) {
        function();
    }

    volatile std::size_t result_sink = 0;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < repetitions; ++i) {
        const auto clusters = function();
        result_sink += clusters.size();
    }
    const auto end = std::chrono::steady_clock::now();
    (void)result_sink;

    return std::chrono::duration<double, std::milli>(end - start).count() /
        static_cast<double>(repetitions);
}

std::vector<detector_core::Point2D> make_points(std::size_t count) {
    constexpr std::size_t cluster_count = 10;
    constexpr double spacing = 0.04;

    std::vector<detector_core::Point2D> points;
    points.reserve(count);

    for (std::size_t index = 0; index < count; ++index) {
        const std::size_t cluster = index % cluster_count;
        const std::size_t local = index / cluster_count;
        const std::size_t row = local / 32;
        const std::size_t column = local % 32;

        points.push_back({
            static_cast<double>(cluster) * 5.0 + static_cast<double>(column) * spacing,
            static_cast<double>(row) * spacing});
    }

    return points;
}

std::vector<detector_core::Cluster> canonicalize(
    std::vector<detector_core::Cluster> clusters) {
    for (auto& cluster : clusters) {
        std::sort(cluster.begin(), cluster.end());
    }
    std::sort(clusters.begin(), clusters.end());
    return clusters;
}

int repetitions_for(std::size_t point_count) {
    if (point_count <= 720) return 500;
    if (point_count <= 2048) return 100;
    return 20;
}

} // namespace

int main() {
    constexpr double tolerance = 0.09;
    constexpr std::size_t min_cluster_size = 2;
    constexpr std::size_t max_cluster_size = 100000;
    const std::vector<std::size_t> sizes{360, 640, 1080, 2048, 5000};

    std::cout << "2D clustering benchmark (same input, same result)\n";
    std::cout << "points,clusters,brute_force_ms,spatial_grid_ms,speedup,correct\n";
    std::cout << std::fixed << std::setprecision(4);

    for (const std::size_t size : sizes) {
        const auto points = make_points(size);
        const auto brute_result = detector_core::euclidean_clusters(
            points, tolerance, min_cluster_size, max_cluster_size);
        const auto grid_result = detector_core::spatial_grid_clusters(
            points, tolerance, min_cluster_size, max_cluster_size);
        const bool correct = canonicalize(brute_result) == canonicalize(grid_result);
        const int repetitions = repetitions_for(size);

        const double brute_ms = average_runtime_ms([&]() {
            return detector_core::euclidean_clusters(
                points, tolerance, min_cluster_size, max_cluster_size);
        }, repetitions);
        const double grid_ms = average_runtime_ms([&]() {
            return detector_core::spatial_grid_clusters(
                points, tolerance, min_cluster_size, max_cluster_size);
        }, repetitions);

        std::cout << size << ',' << brute_result.size() << ','
                  << brute_ms << ',' << grid_ms << ','
                  << (brute_ms / grid_ms) << ','
                  << (correct ? "yes" : "NO") << '\n';

        if (!correct) return 1;
    }

    return 0;
}
