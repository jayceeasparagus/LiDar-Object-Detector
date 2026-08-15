#include "detector_core/kalman_filter_3d.hpp"

namespace detector_core {

KalmanFilter3D::KalmanFilter3D(double process_noise, double measurement_noise)
    : process_noise_(process_noise), measurement_noise_(measurement_noise), initialized_(false) {
    state_.setZero();
    covariance_.setIdentity();
    covariance_ *= 1000.0;
}

void KalmanFilter3D::initialize(double x, double y, double z) {
    state_.setZero();
    state_(0) = x;
    state_(1) = y;
    state_(2) = z;
    covariance_.setIdentity();
    initialized_ = true;
}

void KalmanFilter3D::predict(double delta_time) {
    if (!initialized_ || delta_time <= 0.0) return;
    Eigen::Matrix<double, 6, 6> transition = Eigen::Matrix<double, 6, 6>::Identity();
    transition(0, 3) = delta_time;
    transition(1, 4) = delta_time;
    transition(2, 5) = delta_time;
    state_ = transition * state_;
    covariance_ = transition * covariance_ * transition.transpose();
    covariance_ += Eigen::Matrix<double, 6, 6>::Identity() * process_noise_;
}

void KalmanFilter3D::update(double measured_x, double measured_y, double measured_z) {
    if (!initialized_) {
        initialize(measured_x, measured_y, measured_z);
        return;
    }
    Eigen::Vector3d measurement;
    measurement << measured_x, measured_y, measured_z;
    Eigen::Matrix<double, 3, 6> measurement_matrix;
    measurement_matrix.setZero();
    measurement_matrix(0, 0) = 1.0;
    measurement_matrix(1, 1) = 1.0;
    measurement_matrix(2, 2) = 1.0;
    const Eigen::Matrix3d measurement_covariance =
        Eigen::Matrix3d::Identity() * measurement_noise_;
    const Eigen::Vector3d innovation = measurement - measurement_matrix * state_;
    const Eigen::Matrix3d innovation_covariance =
        measurement_matrix * covariance_ * measurement_matrix.transpose() + measurement_covariance;
    const Eigen::Matrix<double, 6, 3> gain =
        covariance_ * measurement_matrix.transpose() * innovation_covariance.inverse();
    state_ += gain * innovation;
    covariance_ = (Eigen::Matrix<double, 6, 6>::Identity() - gain * measurement_matrix) * covariance_;
}

double KalmanFilter3D::x() const { return state_(0); }
double KalmanFilter3D::y() const { return state_(1); }
double KalmanFilter3D::z() const { return state_(2); }
double KalmanFilter3D::velocity_x() const { return state_(3); }
double KalmanFilter3D::velocity_y() const { return state_(4); }
double KalmanFilter3D::velocity_z() const { return state_(5); }

} // namespace detector_core
