#include "detector_core/kalman_filter_3d.hpp"
#include <gtest/gtest.h>

TEST(KalmanFilter3DTest, InitializesPosition)
{
    detector_core::KalmanFilter3D filter(1.0, 0.1);
    filter.initialize(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(filter.x(), 1.0);
    EXPECT_DOUBLE_EQ(filter.y(), 2.0);
    EXPECT_DOUBLE_EQ(filter.z(), 3.0);
}

TEST(KalmanFilter3DTest, EstimatesVelocityFromMotion)
{
    detector_core::KalmanFilter3D filter(0.01, 0.01);
    filter.initialize(0.0, 0.0, 0.0);
    filter.predict(1.0);
    filter.update(1.0, 2.0, 3.0);
    EXPECT_GT(filter.velocity_x(), 0.0);
    EXPECT_GT(filter.velocity_y(), 0.0);
    EXPECT_GT(filter.velocity_z(), 0.0);
}
