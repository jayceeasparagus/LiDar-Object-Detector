#include "detector_core/multi_object_tracker_3d.hpp"
#include <algorithm>
#include <limits>

namespace detector_core {

MultiObjectTracker3D::MultiObjectTracker3D(double association_distance,
    std::size_t max_missed_frames, double process_noise, double measurement_noise)
    : association_distance_(association_distance), max_missed_frames_(max_missed_frames),
      process_noise_(process_noise), measurement_noise_(measurement_noise), next_track_id_(0) {}

void MultiObjectTracker3D::update(const std::vector<BoundingBox3D>& detections, double delta_time) {
    for (TrackedObstacle3D& track : tracks_) track.predict(delta_time);
    std::vector<bool> track_matched(tracks_.size(), false);
    std::vector<bool> detection_matched(detections.size(), false);
    const double max_distance_squared = association_distance_ * association_distance_;

    for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
        const BoundingBox3D& detection = detections[detection_index];
        std::size_t best_track = tracks_.size();
        double best_distance_squared = std::numeric_limits<double>::max();
        for (std::size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
            if (track_matched[track_index]) continue;
            const double dx = detection.center_x - tracks_[track_index].x();
            const double dy = detection.center_y - tracks_[track_index].y();
            const double dz = detection.center_z - tracks_[track_index].z();
            const double distance_squared = dx * dx + dy * dy + dz * dz;
            if (distance_squared < best_distance_squared) {
                best_distance_squared = distance_squared;
                best_track = track_index;
            }
        }
        if (best_track < tracks_.size() && best_distance_squared <= max_distance_squared) {
            tracks_[best_track].update(detection);
            track_matched[best_track] = true;
            detection_matched[detection_index] = true;
        }
    }

    for (std::size_t i = 0; i < tracks_.size(); ++i) {
        if (!track_matched[i]) tracks_[i].mark_missed();
    }
    for (std::size_t i = 0; i < detections.size(); ++i) {
        if (!detection_matched[i]) {
            tracks_.emplace_back(next_track_id_++, detections[i], process_noise_, measurement_noise_);
        }
    }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [this](const TrackedObstacle3D& track) {
        return track.missed_frames() > max_missed_frames_;
    }), tracks_.end());
}

const std::vector<TrackedObstacle3D>& MultiObjectTracker3D::tracks() const { return tracks_; }

} // namespace detector_core
