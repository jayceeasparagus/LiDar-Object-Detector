#pragma once

#include "detector_core/bounding_box3d.hpp"
#include "detector_core/kalman_filter_3d.hpp"
#include <cstddef>

namespace detector_core {

class TrackedObstacle3D {
public:
    TrackedObstacle3D(std::size_t id, const BoundingBox3D& initial_box,
        double process_noise, double measurement_noise);
    void predict(double delta_time);
    void update(const BoundingBox3D& measured_box);
    void mark_missed();
    std::size_t id() const;
    double x() const;
    double y() const;
    double z() const;
    double velocity_x() const;
    double velocity_y() const;
    double velocity_z() const;
    std::size_t missed_frames() const;
    const BoundingBox3D& bounding_box() const;

private:
    std::size_t id_;
    KalmanFilter3D filter_;
    BoundingBox3D bounding_box_;
    std::size_t missed_frames_;
};

} // namespace detector_core
