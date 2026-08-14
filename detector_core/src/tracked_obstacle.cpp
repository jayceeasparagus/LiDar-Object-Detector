#include "detector_core/tracked_obstacle.hpp"

namespace detector_core {
    namespace {
        double center_x(const BoundingBox2D& box) {
            return (box.min_x + box.max_x) / 2.0;
        }

        double center_y(const BoundingBox2D& box) {
            return (box.min_y + box.max_y) / 2.0;
        }

        OrientedBoundingBox2D oriented_box_from_axis_aligned(const BoundingBox2D& box) {
            return OrientedBoundingBox2D{
                center_x(box), center_y(box),
                box.max_x - box.min_x, box.max_y - box.min_y, 0.0};
        }
    }

    TrackedObstacle::TrackedObstacle(std::size_t id, const BoundingBox2D& initial_box,
        double process_noise, double measurement_noise)
            : TrackedObstacle(id, initial_box, oriented_box_from_axis_aligned(initial_box),
                process_noise, measurement_noise) {
            }

    TrackedObstacle::TrackedObstacle(std::size_t id, const BoundingBox2D& initial_box,
        const OrientedBoundingBox2D& initial_oriented_box,
        double process_noise, double measurement_noise)
            : id_(id), filter_(process_noise, measurement_noise),
              bounding_box_(initial_box), oriented_bounding_box_(initial_oriented_box),
              missed_frames_(0) {
                filter_.initialize(center_x(initial_box), center_y(initial_box));
            }

        std::size_t TrackedObstacle::id() const {
            return id_;
        }

        double TrackedObstacle::x() const {
            return filter_.x();
        }

        double TrackedObstacle::y() const {
            return filter_.y();
        }

        double TrackedObstacle::velocity_x() const {
            return filter_.velocity_x();
        }

        double TrackedObstacle::velocity_y() const {
            return filter_.velocity_y();
        }

        std::size_t TrackedObstacle::missed_frames() const {
            return missed_frames_;
        }

        const BoundingBox2D& TrackedObstacle::bounding_box() const {
            return bounding_box_;
        }

        const OrientedBoundingBox2D& TrackedObstacle::oriented_bounding_box() const {
            return oriented_bounding_box_;
        }

        void TrackedObstacle::predict(double delta_time) {
            filter_.predict(delta_time);
        }

        void TrackedObstacle::update(const BoundingBox2D& measured_box) {
            update(measured_box, oriented_box_from_axis_aligned(measured_box));
        }

        void TrackedObstacle::update(const BoundingBox2D& measured_box,
            const OrientedBoundingBox2D& measured_oriented_box) {
            bounding_box_ = measured_box;
            oriented_bounding_box_ = measured_oriented_box;

            filter_.update(center_x(measured_box), center_y(measured_box));

            missed_frames_ = 0;
        }

        void TrackedObstacle::mark_missed() {
            ++missed_frames_;
        }
}