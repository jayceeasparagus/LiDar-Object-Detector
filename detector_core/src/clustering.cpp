#include "detector_core/clustering.hpp"
#include <queue>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace detector_core {
    struct GridCell {
        int x;
        int y;

        bool operator==(const GridCell& other) const {
            return x == other.x && y == other.y;
        }
    };

    struct GridCellHash {
        std::size_t operator()(const GridCell& cell) const {
            const std::size_t x_hash = std::hash<int>{}(cell.x);
            const std::size_t y_hash = std::hash<int>{}(cell.y);

            return x_hash ^ (y_hash << 1);
        }
    };

    GridCell point_to_cell(const Point2D& point, double cell_size) {
        return {static_cast<int>(std::floor(point.x / cell_size)), 
        static_cast<int>(std::floor(point.y / cell_size))};
    }

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

    std::vector<Cluster> ordered_euclidean_clusters(const std::vector<Point2D>& points, double distance_tolerance,
        std::size_t min_cluster_size, std::size_t max_cluster_size) {
            std::vector<Cluster> clusters;

            if (points.empty()) {
                return clusters;
            }

            const double tolerance_squared = distance_tolerance * distance_tolerance;

            Cluster current_cluster;
            current_cluster.push_back(0);

            const auto save_current_cluster = [&]() {
                if (current_cluster.size() >= min_cluster_size && current_cluster.size() <= max_cluster_size) {
                    clusters.push_back(current_cluster);
                }

                current_cluster.clear();
            };

            for (std::size_t i = 1; i < points.size(); ++i) {
                const Point2D& previous = points[i - 1];
                const Point2D& current = points[i];

                const double dx = current.x - previous.x;
                const double dy = current.y - previous.y;
                const double distance_squared = dx * dx + dy * dy;

                if (distance_squared <= tolerance_squared) {
                    current_cluster.push_back(i);
                } else {
                    save_current_cluster();
                    current_cluster.push_back(i);
                }
            }
        
            save_current_cluster();

            return clusters;
        };

    std::vector<std::vector<Point2D>> spatial_grid_clusters(
        const std::vector<Point2D>& points, double distance_tolerance,
        std::size_t min_cluster_size, std::size_t max_cluster_size) {
            using Grid = std::unordered_map<GridCell, std::vector<std::size_t>, GridCellHash>;

            Grid grid;
            grid.reserve(points.size());

            for (std::size_t i = 0; i < points.size(); ++i) {
                const GridCell cell = point_to_cell(points[i], distance_tolerance);


                grid[cell].push_back(i);
            }

            std::vector<std::vector<Point2D>> clusters;
            
            if (points.empty() || distance_tolerance <= 0.0) {
                return clusters;
            }

            const double tolerance_squared = distance_tolerance * distance_tolerance;

            std::vector<bool> visited(points.size(), false);

            for (std::size_t seed = 0; seed < points.size(); ++seed) {
                if (visited[seed]) {
                    continue;
                }

                std::queue<std::size_t> pending_points;
                std::vector<std::size_t> cluster_indices;

                visited[seed] = true;
                pending_points.push(seed);

                while (!pending_points.empty()) {
                    const std::size_t current_index = pending_points.front();
                    pending_points.pop();

                    cluster_indices.push_back(current_index);

                    const GridCell current_cell = point_to_cell(points[current_index], distance_tolerance);

                    for (int offset_x = -1; offset_x <= 1; ++offset_x) {
                        for (int offset_y = -1; offset_y <= 1; ++offset_y) {
                            const GridCell neighbor_cell{
                                current_cell.x + offset_x, current_cell.y + offset_y
                            };

                            const auto cell_iterator = grid.find(neighbor_cell);

                            if (cell_iterator == grid.end()) {
                                continue;
                            }

                            for (const std::size_t candidate_index : cell_iterator->second) {
                                if (visited[candidate_index]) {
                                    continue;
                                }

                                const double difference_x = points[candidate_index].x - points[current_index].x;
                                const double difference_y = points[candidate_index].y - points[current_index].y;
                                const double distance_squared = difference_x * difference_x + difference_y * difference_y;

                                if (distance_squared <= tolerance_squared) {
                                    visited[candidate_index] = true;
                                    pending_points.push(candidate_index);
                                }
                            }
                        }
                    }
                }

                if (cluster_indices.size() >= min_cluster_size && cluster_indices.size() <= max_cluster_size) {
                    std::vector<Point2D> cluster;

                    for (const std::size_t point_index : cluster_indices) {
                        cluster.push_back(points[point_index]);
                    }

                    clusters.push_back(cluster);
                }
            }

            return clusters;
        }
} // namespace detector_core