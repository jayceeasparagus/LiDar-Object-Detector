#include "detector_core/clustering.hpp"
#include <chrono>
#include <cstddef>
#include <iostream>
#include <vector>

template <typename Function>
double average_runtime_ms(Function function, int repetitions)
{
    for (int i = 0; i < 10; ++i) {
        function();
    }

    volatile std::size_t result_sink = 0;

    const auto start = std::chrono::steady_clock::now();

    for (int i = 0; i < repetitions; ++i) {
        const auto clusters = function();
        result_sink += clusters.size();
    }

    const auto end = std::chrono::steady_clock::now();

    const double total_ms =
        std::chrono::duration<double, std::milli>(
            end - start).count();

    return total_ms / repetitions;
}

namespace {

std::vector<detector_core::Point2D> make_points(
    std::size_t count) {
    std::vector<detector_core::Point2D> points;
    points.reserve(count);

    constexpr std::size_t cluster_count = 10;
    const std::size_t points_per_cluster =
        count / cluster_count;

    for (std::size_t cluster = 0;
         cluster < cluster_count;
         ++cluster) {
        for (std::size_t local_index = 0;
             local_index < points_per_cluster;
             ++local_index) {
            points.push_back(detector_core::Point2D{
                static_cast<double>(cluster) * 5.0 +
                    static_cast<double>(local_index) * 0.01,
                static_cast<double>(local_index) * 0.01
            });
        }
    }

    while (points.size() < count) {
        points.push_back(points.back());
    }

    return points;
}

}  // namespace

int main() {
    constexpr int repetitions = 1000;

    const std::vector<std::size_t> sizes{
        360,
        720,
        1080,
        2048
    };

    for (const std::size_t size : sizes) {
    const std::vector<detector_core::Point2D> points =
        make_points(size);

    constexpr double distance_tolerance = 0.2;
    constexpr std::size_t min_cluster_size = 2;
    constexpr std::size_t max_cluster_size = 100000;

    const auto brute_force_clusters =
        detector_core::euclidean_clusters(
            points,
            distance_tolerance,
            min_cluster_size,
            max_cluster_size);

    const auto grid_clusters =
        detector_core::spatial_grid_clusters(
            points,
            distance_tolerance,
            min_cluster_size,
            max_cluster_size);

    const double brute_force_ms =
        average_runtime_ms(
            [&]() {
                return detector_core::euclidean_clusters(
                    points,
                    distance_tolerance,
                    min_cluster_size,
                    max_cluster_size);
            },
            repetitions);

    const double grid_ms =
        average_runtime_ms(
            [&]() {
                return detector_core::spatial_grid_clusters(
                    points,
                    distance_tolerance,
                    min_cluster_size,
                    max_cluster_size);
            },
            repetitions);

    std::cout
        << "points=" << size
        << " brute_force_clusters="
        << brute_force_clusters.size()
        << " brute_force_ms="
        << brute_force_ms
        << " grid_clusters="
        << grid_clusters.size()
        << " grid_ms="
        << grid_ms
        << '\n';
}

    return 0;
}