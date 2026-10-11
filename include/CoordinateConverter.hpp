#pragma once

#include <cmath>

// Conversion helpers for the 2D Cartesian and polar coordinates used by the
// radar measurement model. Bearings are measured counter-clockwise from the
// positive x-axis, in radians.
class CoordinateConverter {
public:
    struct Polar {
        double range;
        double bearing;
    };

    struct Cartesian {
        double x;
        double y;
    };

    static Polar cartesianToPolar(double x, double y) {
        return {std::hypot(x, y), std::atan2(y, x)};
    }

    static Cartesian polarToCartesian(double range, double bearing) {
        return {range * std::cos(bearing), range * std::sin(bearing)};
    }

    // Wrap an angle in radians to the interval [-pi, pi).
    static double normalizeAngle(double angle) {
        const double pi = std::acos(-1.0);
        const double two_pi = 2.0 * pi;
        angle = std::fmod(angle + pi, two_pi);
        if (angle < 0.0) {
            angle += two_pi;
        }
        return angle - pi;
    }
};
