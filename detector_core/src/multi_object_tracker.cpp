#include "detector_core/multi_object_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace detector_core {

namespace {
    double box_center_x(const BoundingBox2D& box) {
        return (box.min_x + box.max_x) / 2.0;
    }

    double box_center_y(const BoundingBox2D& box) {
        return (box.min_y + box.max_y) / 2.0;
    }
}

MultiObjectTracker::MultiObjectTracker(double association_distance, std::size_t max_missed_frames, double process_noise, double measurement_noise)
    : association_distance_(association_distance), max_missed_frames_(max_missed_frames), process_noise_(process_noise), measurement_noise_(measurement_noise),
      next_track_id_(0) {
}

void MultiObjectTracker::update(
    const std::vector<BoundingBox2D>& detections, double delta_time) {
    std::vector<OrientedBoundingBox2D> oriented_detections;
    oriented_detections.reserve(detections.size());
    for (const BoundingBox2D& detection : detections) {
        oriented_detections.push_back(OrientedBoundingBox2D{
            box_center_x(detection), box_center_y(detection),
            detection.max_x - detection.min_x,
            detection.max_y - detection.min_y, 0.0});
    }
    update(detections, oriented_detections, delta_time);
}

void MultiObjectTracker::update(
    const std::vector<BoundingBox2D>& detections,
    const std::vector<OrientedBoundingBox2D>& oriented_detections,
    double delta_time) {
    if (tracks_.empty()) {
        for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
            const OrientedBoundingBox2D oriented_box = oriented_detections[detection_index];
            tracks_.emplace_back(next_track_id_, detections[detection_index], oriented_box,
                process_noise_, measurement_noise_);

            ++next_track_id_;
        }

        return;
    }

    for (TrackedObstacle& track : tracks_) {
        track.predict(delta_time);
    }

    std::vector<bool> track_matched(tracks_.size(),false );

    std::vector<bool> detection_matched(detections.size(), false);

    const double maximum_distance_squared = association_distance_ * association_distance_;

    for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {

        const double detection_x = box_center_x(detections[detection_index]);

        const double detection_y = box_center_y(detections[detection_index]);

        std::size_t best_track_index = tracks_.size();

        double best_distance_squared = std::numeric_limits<double>::max();

        for (std::size_t track_index = 0; track_index < tracks_.size(); ++track_index) {

            if (track_matched[track_index]) {
                continue;
            }

            const double difference_x = detection_x - tracks_[track_index].x();

            const double difference_y = detection_y - tracks_[track_index].y();

            const double distance_squared = difference_x * difference_x + difference_y * difference_y;

            if (distance_squared < best_distance_squared) {
                best_distance_squared = distance_squared;
                best_track_index = track_index;
            }
        }

        if (best_track_index < tracks_.size() && best_distance_squared <= maximum_distance_squared) {
            const OrientedBoundingBox2D oriented_box = oriented_detections[detection_index];
            tracks_[best_track_index].update(detections[detection_index], oriented_box);

            track_matched[best_track_index] = true;
            detection_matched[detection_index] = true;
        }
    }

    for (std::size_t track_index = 0;
         track_index < tracks_.size();
         ++track_index) {

        if (!track_matched[track_index]) {
            tracks_[track_index].mark_missed();
        }
    }

    for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {

        if (!detection_matched[detection_index]) {
            const OrientedBoundingBox2D oriented_box = oriented_detections[detection_index];
            tracks_.emplace_back(next_track_id_, detections[detection_index], oriented_box,
                process_noise_, measurement_noise_);

            ++next_track_id_;
        }
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [this](const TrackedObstacle& track) {
        return track.missed_frames() >max_missed_frames_;}),tracks_.end());
}

const std::vector<TrackedObstacle>&
MultiObjectTracker::tracks() const
{
    return tracks_;
}

}