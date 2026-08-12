#include "detector_core/kalman_filter_2d.hpp"
#include <gtest/gtest.h>

TEST(KalmanFilter2DTest, InitializesPositionAndZeroVelocity)
{
    detector_core::KalmanFilter2D filter(1.0, 1.0);

    filter.initialize(3.0, 4.0);

    EXPECT_DOUBLE_EQ(filter.x(), 3.0);
    EXPECT_DOUBLE_EQ(filter.y(), 4.0);
    EXPECT_DOUBLE_EQ(filter.velocity_x(), 0.0);
    EXPECT_DOUBLE_EQ(filter.velocity_y(), 0.0);
}

TEST(KalmanFilter2DTest, PredictionDoesNotMoveStationaryObject)
{
    detector_core::KalmanFilter2D filter(1.0, 1.0);

    filter.initialize(3.0, 4.0);
    filter.predict(1.0);

    EXPECT_DOUBLE_EQ(filter.x(), 3.0);
    EXPECT_DOUBLE_EQ(filter.y(), 4.0);
}

TEST(KalmanFilter2DTest, UpdateMovesEstimateTowardMeasurement)
{
    detector_core::KalmanFilter2D filter(1.0, 1.0);

    filter.initialize(0.0, 0.0);
    filter.update(10.0, 10.0);

    EXPECT_GT(filter.x(), 0.0);
    EXPECT_LT(filter.x(), 10.0);

    EXPECT_GT(filter.y(), 0.0);
    EXPECT_LT(filter.y(), 10.0);
}

TEST(KalmanFilter2DTest, EstimatesVelocityFromMovingMeasurements)
{
    detector_core::KalmanFilter2D filter(1.0, 0.01);

    filter.initialize(0.0, 0.0);

    filter.predict(1.0);
    filter.update(1.0, 0.0);

    filter.predict(1.0);
    filter.update(2.0, 0.0);

    EXPECT_GT(filter.velocity_x(), 0.0);
    EXPECT_NEAR(filter.velocity_y(), 0.0, 0.1);
}