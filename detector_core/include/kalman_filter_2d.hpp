#pragma once

#include <Eigen/Dense>

namespace detector_core {
    class KalmanFilter2D {
        public:
            KalmanFilter2D(double process_noise, double measurement_noise);

            void initialize(double x, double y);

            void predict(double delta_time);

            void uptae(double measured_x, double measured_y);

            double x() const;
            double y() const;
            double velocity_x() const;
            double velocity_y() const;

        private:
            Eigen::Vector4d state_;
            Eigen::Matrix4d covariance_;

            double process_noise_;
            double measurement_noise_;

            bool initialized_;
    };
}