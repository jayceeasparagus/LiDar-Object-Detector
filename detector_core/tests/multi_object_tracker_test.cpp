#include "detector_core/multi_object_tracker.hpp"

#include <gtest/gtest.h>

TEST(MultiObjectTrackerTest, StartsWithNoTracks)
{
    detector_core::MultiObjectTracker tracker(
        1.0,
        3,
        1.0,
        0.1);

    EXPECT_TRUE(tracker.tracks().empty());
}

TEST(MultiObjectTrackerTest, CreatesTracksFromInitialDetections)
{
    detector_core::MultiObjectTracker tracker(
        1.0,
        3,
        1.0,
        0.1);

    const std::vector<detector_core::BoundingBox2D> detections{
        {0.0, 0.0, 1.0, 1.0},
        {4.0, 4.0, 5.0, 5.0}
    };

    tracker.update(detections, 0.1);

    ASSERT_EQ(tracker.tracks().size(), 2);
    EXPECT_EQ(tracker.tracks()[0].id(), 0);
    EXPECT_EQ(tracker.tracks()[1].id(), 1);
}

TEST(MultiObjectTrackerTest, MaintainsTrackIdForNearbyDetection)
{
    detector_core::MultiObjectTracker tracker(
        1.0,
        3,
        1.0,
        0.1);

    tracker.update(
        {detector_core::BoundingBox2D{0.0, 0.0, 1.0, 1.0}},
        1.0);

    tracker.update(
        {detector_core::BoundingBox2D{0.2, 0.0, 1.2, 1.0}},
        1.0);

    ASSERT_EQ(tracker.tracks().size(), 1);
    EXPECT_EQ(tracker.tracks()[0].id(), 0);
    EXPECT_EQ(tracker.tracks()[0].missed_frames(), 0);
}

TEST(MultiObjectTrackerTest, DeletesTrackAfterTooManyMisses)
{
    detector_core::MultiObjectTracker tracker(
        1.0,
        2,
        1.0,
        0.1);

    tracker.update(
        {detector_core::BoundingBox2D{0.0, 0.0, 1.0, 1.0}},
        1.0);

    tracker.update({}, 1.0);
    tracker.update({}, 1.0);
    tracker.update({}, 1.0);

    EXPECT_TRUE(tracker.tracks().empty());

    tracker.update(
        {detector_core::BoundingBox2D{5.0, 5.0, 6.0, 6.0}},
        1.0);

    ASSERT_EQ(tracker.tracks().size(), 1);
    EXPECT_EQ(tracker.tracks()[0].id(), 1);
}

TEST(MultiObjectTrackerTest, FarDetectionCreatesNewTrack)
{
    detector_core::MultiObjectTracker tracker(
        1.0,
        3,
        1.0,
        0.1);

    tracker.update(
        {detector_core::BoundingBox2D{0.0, 0.0, 1.0, 1.0}},
        1.0);

    tracker.update(
        {detector_core::BoundingBox2D{5.0, 5.0, 6.0, 6.0}},
        1.0);

    ASSERT_EQ(tracker.tracks().size(), 2);

    EXPECT_EQ(tracker.tracks()[0].id(), 0);
    EXPECT_EQ(tracker.tracks()[0].missed_frames(), 1);

    EXPECT_EQ(tracker.tracks()[1].id(), 1);
    EXPECT_EQ(tracker.tracks()[1].missed_frames(), 0);
}