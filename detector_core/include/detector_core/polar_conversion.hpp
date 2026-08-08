#pragma once

#include "detector_core/point2d.hpp"

namespace detector_core {
    Point2D polar_to_cartesian(double range, double angle_rad);
}