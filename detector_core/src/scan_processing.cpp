#include "detector_core/scan_processing.hpp"
#include "detector_core/polar_conversion.hpp"
#include <cmath>

namespace detector_core {
    std::vector<Point2D> scan_to_points(
        const std::vector<double>& ranges, double angle_min, double angle_increment, double range_min, double range_max) {
            std::vector<Point2D> points;
            points.reserve(ranges.size());

            for (std::size_t i = 0; i < ranges.size(); ++i) {
                const double range = ranges[i];

                if (!std::isfinite(range) || range < range_min || range > range_max) {
                        continue;
                    }

                const double angle = angle_min + static_cast<double>(i) * angle_increment;

                points.push_back(polar_to_cartesian(range, angle));
            }

            return points;
        }
} // namespace detector_core