#pragma once

#include "detector_core/tracked_obstacle.hpp"
#include <cstddef>
#include <vector>

namespace detector_core {
    class MultiObjectTracker {
        public:
            MultiObjectTracker(double association_distance, std::size_t max_missed_frames, 
                double process_noise, double measurement_noise);

            void update(const std::vector<BoundingBox2D>& detections, double delta_time);
            void update(const std::vector<BoundingBox2D>& detections,
                const std::vector<OrientedBoundingBox2D>& oriented_detections,
                double delta_time);

            const std::vector<TrackedObstacle>& tracks() const;

            private:
                double association_distance_;
                std::size_t max_missed_frames_;
                double process_noise_;
                double measurement_noise_;

                std::size_t next_track_id_;

                std::vector<TrackedObstacle> tracks_;
    };
}