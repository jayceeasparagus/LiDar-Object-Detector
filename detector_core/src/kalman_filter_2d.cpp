#include "detector_core/kalman_filter_2d.hpp"

namespace detector_core {
    KalmanFilter2D::KalmanFilter2D(double process_noise, double measurement_noise) : 
        process_noise_(process_noise), measurement_noise_(measurement_noise), initialized_(false) {
            state_.setZero();
            covariance_.setIdentity();
            covariance_ *= 1000.0;
        }

    void KalmanFilter2D::initialize(double x, double y) {
        state_.setZero();
        state_(0) = x;
        state_(1) = y;

        covariance_.setIdentity();
        initialized_ = true;
    }

    double KalmanFilter2D::x() const {
        return state_(0);
    }

    double KalmanFilter2D::y() const {
        return state_(1);
    }

    double KalmanFilter2D::velocity_x() const {
        return state_(2);
    }

    double KalmanFilter2D::velocity_y() const {
        return state_(3);
    }

    void KalmanFilter2D::predict(double delta_time) {
        if (!initialized_ || delta_time <= 0.0) {
            return;
        }

        Eigen::Matrix4d state_transition = Eigen::Matrix4d::Identity();

        state_transition(0, 2) = delta_time;
        state_transition(1, 3) = delta_time;

        state_ = state_transition * state_;

        Eigen::Matrix4d process_covariance = Eigen::Matrix4d::Identity() * process_noise_;

        covariance_ = state_transition * covariance_ * state_transition.transpose() + process_covariance;
    }

    void KalmanFilter2D::update(double measured_x, double measured_y) {
        if (!initialized_) {
            initialize(measured_x, measured_y);
            return;
        }

        Eigen::Vector2d measurement;
        measurement << measured_x, measured_y;

        Eigen::Matrix<double, 2, 4> measurement_matrix;
        measurement_matrix.setZero();
        measurement_matrix(0, 0) = 1.0;
        measurement_matrix(1, 1) = 1.0;

        Eigen::Matrix2d measurement_covariance = Eigen::Matrix2d::Identity() * measurement_noise_;

        const Eigen::Vector2d innovation = measurement - measurement_matrix * state_;

        const Eigen::Matrix2d innovation_covariance = measurement_matrix * covariance_ * measurement_matrix.transpose() + measurement_covariance;

        const Eigen::Matrix<double, 4, 2> kalman_gain = covariance_ * measurement_matrix.transpose() * innovation_covariance.inverse();

        state_ = state_ + kalman_gain * innovation;

        const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();

        covariance_ = (identity - kalman_gain * measurement_matrix) * covariance_;
    }
}