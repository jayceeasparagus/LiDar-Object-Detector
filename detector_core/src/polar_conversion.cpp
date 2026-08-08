#include "detector_core/polar_conversion.hpp"
#include <cmath>

namespace detector_core {
    Point2D polar_to_cartesian(double range, double angle_rad) {
        return Point2D{range * std::cos(angle_rad), range * std::sin(angle_rad)};
    }
} // namespace detector_core