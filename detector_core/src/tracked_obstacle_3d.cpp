#include "detector_core/tracked_obstacle_3d.hpp"

namespace detector_core {

TrackedObstacle3D::TrackedObstacle3D(std::size_t id, const BoundingBox3D& initial_box,
    double process_noise, double measurement_noise)
    : id_(id), filter_(process_noise, measurement_noise),
      bounding_box_(initial_box), missed_frames_(0) {
    filter_.initialize(initial_box.center_x, initial_box.center_y, initial_box.center_z);
}

void TrackedObstacle3D::predict(double delta_time) { filter_.predict(delta_time); }
void TrackedObstacle3D::update(const BoundingBox3D& measured_box) {
    bounding_box_ = measured_box;
    filter_.update(measured_box.center_x, measured_box.center_y, measured_box.center_z);
    missed_frames_ = 0;
}
void TrackedObstacle3D::mark_missed() { ++missed_frames_; }
std::size_t TrackedObstacle3D::id() const { return id_; }
double TrackedObstacle3D::x() const { return filter_.x(); }
double TrackedObstacle3D::y() const { return filter_.y(); }
double TrackedObstacle3D::z() const { return filter_.z(); }
double TrackedObstacle3D::velocity_x() const { return filter_.velocity_x(); }
double TrackedObstacle3D::velocity_y() const { return filter_.velocity_y(); }
double TrackedObstacle3D::velocity_z() const { return filter_.velocity_z(); }
std::size_t TrackedObstacle3D::missed_frames() const { return missed_frames_; }
const BoundingBox3D& TrackedObstacle3D::bounding_box() const { return bounding_box_; }

} // namespace detector_core
