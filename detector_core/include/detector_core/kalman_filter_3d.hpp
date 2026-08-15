#pragma once

#include <Eigen/Dense>

namespace detector_core {

class KalmanFilter3D {
public:
    KalmanFilter3D(double process_noise, double measurement_noise);
    void initialize(double x, double y, double z);
    void predict(double delta_time);
    void update(double measured_x, double measured_y, double measured_z);
    double x() const;
    double y() const;
    double z() const;
    double velocity_x() const;
    double velocity_y() const;
    double velocity_z() const;

private:
    Eigen::Matrix<double, 6, 1> state_;
    Eigen::Matrix<double, 6, 6> covariance_;
    double process_noise_;
    double measurement_noise_;
    bool initialized_;
};

} // namespace detector_core
