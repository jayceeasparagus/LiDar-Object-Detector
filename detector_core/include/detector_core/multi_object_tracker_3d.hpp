#pragma once

#include "detector_core/tracked_obstacle_3d.hpp"
#include <cstddef>
#include <vector>

namespace detector_core {

class MultiObjectTracker3D {
public:
    MultiObjectTracker3D(double association_distance, std::size_t max_missed_frames,
        double process_noise, double measurement_noise);
    void update(const std::vector<BoundingBox3D>& detections, double delta_time);
    const std::vector<TrackedObstacle3D>& tracks() const;

private:
    double association_distance_;
    std::size_t max_missed_frames_;
    double process_noise_;
    double measurement_noise_;
    std::size_t next_track_id_;
    std::vector<TrackedObstacle3D> tracks_;
};

} // namespace detector_core
