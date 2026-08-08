#pragma once

namespace detector_core {
    struct BoundingBox2D {
        double min_x;
        double min_y;
        double max_x;
        double max_y;
    };
} // namespace detector_core