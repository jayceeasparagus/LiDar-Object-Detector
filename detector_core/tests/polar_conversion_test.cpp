#include "detector_core/polar_conversion.hpp"
#include <cmath>
#include <gtest/gtest.h>

TEST(PolarConversionTest, ConvertsZeroAngle) {
    const detector_core::Point2D point = detector_core::polar_to_cartesian(2.0, 0.0);

    EXPECT_NEAR(point.x, 2.0, 1e-9);
    EXPECT_NEAR(point.y, 0.0, 1e-9);
}

TEST(PolarConversionTest, ConvertsNinetyDegreeAngle) {
    constexpr double pi = 3.14159265358979323846;

    const detector_core::Point2D point = detector_core::polar_to_cartesian(2.0, pi / 2.0);

    EXPECT_NEAR(point.x, 0.0, 1e-9);
    EXPECT_NEAR(point.y, 2.0, 1e-9);
}