#pragma once

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <vector>

namespace mad::common {

struct Vec2 {
    double x {0.0};
    double y {0.0};

    Vec2 operator+(const Vec2& rhs) const { return {x + rhs.x, y + rhs.y}; }
    Vec2 operator-(const Vec2& rhs) const { return {x - rhs.x, y - rhs.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
};

inline double Dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline double Norm(const Vec2& v) { return std::sqrt(Dot(v, v)); }
inline double Distance(const Vec2& a, const Vec2& b) { return Norm(a - b); }
inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kTwoPi = 2.0 * kPi;

inline double Clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}
inline double NormalizeAngle(double angle) {
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    while (angle < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

enum class ActorType {
    Ego,
    Vehicle,
};

struct Waypoint {
    double x {0.0};
    double y {0.0};
    double speed_limit {22.0};
};

struct TrajectoryPoint {
    double t {0.0};
    double x {0.0};
    double y {0.0};
    double yaw {0.0};
    double target_speed {0.0};
};

inline std::ostream& operator<<(std::ostream& os, const Vec2& value) {
    os << "(" << value.x << ", " << value.y << ")";
    return os;
}

} // namespace mad::common
