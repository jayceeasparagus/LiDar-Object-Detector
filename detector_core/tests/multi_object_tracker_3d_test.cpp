#include "detector_core/multi_object_tracker_3d.hpp"
#include <gtest/gtest.h>

namespace {
detector_core::BoundingBox3D box(double x, double y, double z) {
    return {x, y, z, 1.0, 1.0, 1.0, 0.0};
}
}

TEST(MultiObjectTracker3DTest, CreatesPersistentIds)
{
    detector_core::MultiObjectTracker3D tracker(1.0, 2, 0.01, 0.1);
    tracker.update({box(0.0, 0.0, 0.0), box(5.0, 0.0, 0.0)}, 0.1);
    ASSERT_EQ(tracker.tracks().size(), 2);
    const auto first_id = tracker.tracks()[0].id();
    const auto second_id = tracker.tracks()[1].id();

    tracker.update({box(5.1, 0.0, 0.0), box(0.1, 0.0, 0.0)}, 0.1);
    ASSERT_EQ(tracker.tracks().size(), 2);
    EXPECT_EQ(tracker.tracks()[0].id(), first_id);
    EXPECT_EQ(tracker.tracks()[1].id(), second_id);
}

TEST(MultiObjectTracker3DTest, RemovesMissedTrack)
{
    detector_core::MultiObjectTracker3D tracker(1.0, 1, 0.01, 0.1);
    tracker.update({box(0.0, 0.0, 0.0)}, 0.1);
    tracker.update({}, 0.1);
    tracker.update({}, 0.1);
    EXPECT_TRUE(tracker.tracks().empty());
}
