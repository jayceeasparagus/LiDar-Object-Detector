#include "detector_core/bounding_box3d_processing.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>

namespace detector_core {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

BoundingBox3D bounding_box3d_for_cluster(
    const std::vector<Point3D>& points,
    const std::vector<std::size_t>& cluster_indices) {
    if (cluster_indices.empty()) {
        return BoundingBox3D{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    double center_x = 0.0;
    double center_y = 0.0;
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();

    for (const std::size_t index : cluster_indices) {
        const Point3D& point = points[index];
        center_x += point.x;
        center_y += point.y;
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }

    const double point_count = static_cast<double>(cluster_indices.size());
    center_x /= point_count;
    center_y /= point_count;

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

    covariance_xx /= point_count;
    covariance_xy /= point_count;
    covariance_yy /= point_count;

    Eigen::Matrix2d covariance;
    covariance << covariance_xx, covariance_xy,
                  covariance_xy, covariance_yy;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    const Eigen::Vector2d minor_axis = solver.eigenvectors().col(0);
    const Eigen::Vector2d major_axis = solver.eigenvectors().col(1);

    double min_major = std::numeric_limits<double>::max();
    double max_major = std::numeric_limits<double>::lowest();
    double min_minor = std::numeric_limits<double>::max();
    double max_minor = std::numeric_limits<double>::lowest();

    for (const std::size_t index : cluster_indices) {
        const Eigen::Vector2d relative_point(
            points[index].x - center_x,
            points[index].y - center_y);
        const double major_projection = relative_point.dot(major_axis);
        const double minor_projection = relative_point.dot(minor_axis);
        min_major = std::min(min_major, major_projection);
        max_major = std::max(max_major, major_projection);
        min_minor = std::min(min_minor, minor_projection);
        max_minor = std::max(max_minor, minor_projection);
    }

    const double major_middle = (min_major + max_major) / 2.0;
    const double minor_middle = (min_minor + max_minor) / 2.0;
    const Eigen::Vector2d box_center =
        Eigen::Vector2d(center_x, center_y) +
        major_axis * major_middle + minor_axis * minor_middle;

    double yaw = std::atan2(major_axis.y(), major_axis.x());
    while (yaw > kPi / 2.0) {
        yaw -= kPi;
    }
    while (yaw < -kPi / 2.0) {
        yaw += kPi;
    }

    return BoundingBox3D{
        box_center.x(),
        box_center.y(),
        (min_z + max_z) / 2.0,
        max_major - min_major,
        max_minor - min_minor,
        max_z - min_z,
        yaw};
}

}  // namespace detector_core
