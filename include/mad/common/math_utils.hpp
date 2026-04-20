#pragma once

#include "mad/common/types.hpp"

namespace mad::common {

inline double Saturate(double value) {
    return Clamp(value, 0.0, 1.0);
}

inline double Lerp(double a, double b, double alpha) {
    return a + (b - a) * alpha;
}

inline double SmoothStep(double alpha) {
    const double t = Saturate(alpha);
    return t * t * (3.0 - 2.0 * t);
}

} // namespace mad::common
