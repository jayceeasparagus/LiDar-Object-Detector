#include "detector_core/clustering.hpp"
#include <queue>

namespace detector_core {
    std::vector<Cluster> euclidean_clusters(const std::vector<Point2D>& points, double distance_tolerance,
        std::size_t min_cluster_size, std::size_t max_cluster_size) {
            std::vector<Cluster> clusters;
            std::vector<bool> visited(points.size(), false);

            const double tolerance_squared = distance_tolerance * distance_tolerance;

            for (std::size_t i = 0; i < points.size(); ++i) {
                if (visited[i]) {
                    continue;
                }

                Cluster cluster;
                std::queue<std::size_t> search_queue;

                visited[i] = true;
                search_queue.push(i);

                while (!search_queue.empty()) {
                    const std::size_t current = search_queue.front();
                    search_queue.pop();

                    cluster.push_back(current);

                    for (std::size_t near = 0; near < points.size(); ++near) {
                        if (visited[near]) {
                            continue;
                        }

                        const double dx = points[current].x - points[near].x;
                        const double dy = points[current].y - points[near].y;
                        const double distance_squared = dx * dx + dy * dy;

                        if (distance_squared <= tolerance_squared) {
                            visited[near] = true;
                            search_queue.push(near);
                        }
                    }
                }

                if (cluster.size() >= min_cluster_size && cluster.size() <= max_cluster_size) {
                    clusters.push_back(cluster);
                }
            }

            return clusters;
        }
} // namespace detector_core