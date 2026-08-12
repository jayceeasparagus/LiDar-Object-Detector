#include "detector_core/tracked_obstacle.hpp"

namespace detector_core {
    namespace {
        double center_x(const BoundingBox2D& box) {
            return (box.min_x + box.max_x) / 2.0;
        }

        double center_y(const BoundingBox2D& box) {
            return (box.min_y + box.max_y) / 2.0;
        }
    }

    TrackedObstacle::TrackedObstacle(std::size_t id, const BoundingBox2D& initial_box,
        double process_noise, double measurement_noise)
            : id_(id), filter_(process_noise, measurement_noise), bounding_box_(initial_box), missed_frames_(0) {
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

        void TrackedObstacle::predict(double delta_time) {
            filter_.predict(delta_time);
        }

        void TrackedObstacle::update(const BoundingBox2D& measured_box) {
            bounding_box_ = measured_box;

            filter_.update(center_x(measured_box), center_y(measured_box));

            missed_frames_ = 0;
        }

        void TrackedObstacle::mark_missed() {
            ++missed_frames_;
        }
}