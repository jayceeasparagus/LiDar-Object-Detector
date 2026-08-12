#pragma once

#include "detector_core/bounding_box.hpp"
#include "detector_core/kalman_filter_2d.hpp"
#include <cstddef>

namespace detector_core {
    class TrackedObstacle {
        public:
            TrackedObstacle(std::size_t id, const BoundingBox2D& initial_box,
                double process_noise, double measurement_noise);

            void predict(double delta_time);

            void update(const BoundingBox2D& measured_box);

            void mark_missed();

            std::size_t id() const;
            double x() const;
            double y() const;
            double velocity_x() const;
            double velocity_y() const;
            std::size_t missed_frames() const;
            const BoundingBox2D& bounding_box() const;

            private:
                std::size_t id_;
                KalmanFilter2D filter_;
                BoundingBox2D bounding_box_;
                std::size_t missed_frames_;
    };
}