#include "detector_core/bounding_box.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <limits>

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
            if (point.x > box.max_x) {
                box.max_x = point.x;
            }
            if (point.y > box.max_y) {
                box.max_y = point.y;
            }
        }

        return box;
    }

    OrientedBoundingBox2D oriented_bounding_box_for_cluster(
    const std::vector<Point2D>& points,
    const std::vector<std::size_t>& cluster_indices) {

    if (cluster_indices.empty()) {
        return OrientedBoundingBox2D{0.0, 0.0, 0.0, 0.0, 0.0};
    }

    double center_x = 0.0;
    double center_y = 0.0;

    for (const std::size_t index : cluster_indices) {
        center_x += points[index].x;
        center_y += points[index].y;
    }

    center_x /= static_cast<double>(cluster_indices.size());
    center_y /= static_cast<double>(cluster_indices.size());

    double covariance_xx = 0.0;
    double covariance_xy = 0.0;
    double covariance_yy = 0.0;

    for (const std::size_t index : cluster_indices) {
        const double dx = points[index].x - center_x;
        const double dy = points[index].y - center_y;

        covariance_xx += dx * dx;
        covariance_xy += dx * dy;
        covariance_yy += dy * dy;
    }

    const double point_count =
        static_cast<double>(cluster_indices.size());

    covariance_xx /= point_count;
    covariance_xy /= point_count;
    covariance_yy /= point_count;

    Eigen::Matrix2d covariance;
    covariance << covariance_xx, covariance_xy,
                  covariance_xy, covariance_yy;

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);

    const Eigen::Vector2d minor_axis =
        solver.eigenvectors().col(0);

    const Eigen::Vector2d major_axis =
        solver.eigenvectors().col(1);

    double min_major = std::numeric_limits<double>::max();
    double max_major = std::numeric_limits<double>::lowest();
    double min_minor = std::numeric_limits<double>::max();
    double max_minor = std::numeric_limits<double>::lowest();

    for (const std::size_t index : cluster_indices) {
        const Eigen::Vector2d relative_point(
            points[index].x - center_x,
            points[index].y - center_y);

        const double major_projection =
            relative_point.dot(major_axis);

        const double minor_projection =
            relative_point.dot(minor_axis);

        if (major_projection < min_major) {
            min_major = major_projection;
        }

        if (major_projection > max_major) {
            max_major = major_projection;
        }

        if (minor_projection < min_minor) {
            min_minor = minor_projection;
        }

        if (minor_projection > max_minor) {
            max_minor = minor_projection;
        }
    }

    const double width = max_major - min_major;
    const double height = max_minor - min_minor;

    const double major_middle =
        (min_major + max_major) / 2.0;

    const double minor_middle =
        (min_minor + max_minor) / 2.0;

    const Eigen::Vector2d box_center =
        Eigen::Vector2d(center_x, center_y)
        + major_axis * major_middle
        + minor_axis * minor_middle;

    double yaw = std::atan2(major_axis.y(), major_axis.x());

    constexpr double pi = 3.14159265358979323846;

    while (yaw > pi / 2.0) {
        yaw -= pi;
    }

    while (yaw < -pi / 2.0) {
        yaw += pi;
    }

    return OrientedBoundingBox2D{
        box_center.x(),
        box_center.y(),
        width,
        height,
        yaw
    };
}
} // namespace detector_core