#include "detector_core/clustering.hpp"
#include <chrono>
#include <cstddef>
#include <iostream>
#include <vector>

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
    const std::vector<std::size_t> sizes{
        360,
        720,
        1080,
        2048
    };

    for (const std::size_t size : sizes) {
        const std::vector<detector_core::Point2D> points =
            make_points(size);

        const auto brute_force_start =
        std::chrono::steady_clock::now();

        const std::vector<detector_core::Cluster>
            brute_force_clusters =
                detector_core::euclidean_clusters(
                    points,
                    0.2,
                    2,
                    100000);

        const auto brute_force_end =
            std::chrono::steady_clock::now();

        const auto ordered_start =
            std::chrono::steady_clock::now();

        const std::vector<detector_core::Cluster>
            ordered_clusters =
                detector_core::ordered_euclidean_clusters(
                    points,
                    0.2,
                    2,
                    100000);

        const auto ordered_end =
            std::chrono::steady_clock::now();

        const double brute_force_ms =
            std::chrono::duration<double, std::milli>(
                brute_force_end - brute_force_start).count();

        const double ordered_ms =
            std::chrono::duration<double, std::milli>(
                ordered_end - ordered_start).count();

        std::cout
            << "points=" << size
            << " brute_force_clusters="
            << brute_force_clusters.size()
            << " brute_force_ms="
            << brute_force_ms
            << " ordered_clusters="
            << ordered_clusters.size()
            << " ordered_ms="
            << ordered_ms
            << '\n';
            }

    return 0;
}