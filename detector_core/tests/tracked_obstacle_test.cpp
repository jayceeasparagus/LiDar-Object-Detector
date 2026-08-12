#include "detector_core/tracked_obstacle.hpp"

#include <gtest/gtest.h>

TEST(TrackedObstacleTest, InitializesTrackFromBoundingBox)
{
    const detector_core::BoundingBox2D box{
        2.0,
        4.0,
        6.0,
        8.0
    };

    const detector_core::TrackedObstacle track(
        7,
        box,
        1.0,
        0.1);

    EXPECT_EQ(track.id(), 7);
    EXPECT_DOUBLE_EQ(track.x(), 4.0);
    EXPECT_DOUBLE_EQ(track.y(), 6.0);
    EXPECT_DOUBLE_EQ(track.velocity_x(), 0.0);
    EXPECT_DOUBLE_EQ(track.velocity_y(), 0.0);
    EXPECT_EQ(track.missed_frames(), 0);
}

TEST(TrackedObstacleTest, UpdateReplacesBoxAndResetsMissedFrames)
{
    const detector_core::BoundingBox2D initial_box{
        0.0,
        0.0,
        1.0,
        1.0
    };

    detector_core::TrackedObstacle track(
        3,
        initial_box,
        1.0,
        0.1);

    track.mark_missed();
    track.mark_missed();

    const detector_core::BoundingBox2D updated_box{
        2.0,
        4.0,
        4.0,
        8.0
    };

    track.update(updated_box);

    EXPECT_EQ(track.missed_frames(), 0);
    EXPECT_DOUBLE_EQ(track.bounding_box().min_x, 2.0);
    EXPECT_DOUBLE_EQ(track.bounding_box().min_y, 4.0);
    EXPECT_DOUBLE_EQ(track.bounding_box().max_x, 4.0);
    EXPECT_DOUBLE_EQ(track.bounding_box().max_y, 8.0);
}

TEST(TrackedObstacleTest, CountsMissedFrames)
{
    const detector_core::BoundingBox2D box{
        0.0,
        0.0,
        1.0,
        1.0
    };

    detector_core::TrackedObstacle track(
        1,
        box,
        1.0,
        0.1);

    track.mark_missed();
    track.mark_missed();

    EXPECT_EQ(track.missed_frames(), 2);
}