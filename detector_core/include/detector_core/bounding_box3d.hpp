#pragma once

namespace detector_core {
    struct BoundingBox3D {
        double center_x;
        double center_y;
        double center_z;

        double width;
        double depth;
        double height;

        double yaw;
    };
} // namespace detector_core