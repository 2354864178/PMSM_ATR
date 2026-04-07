#pragma once

#include <algorithm>
#include <cmath>

namespace numeric_utils {

inline double clamp_floor(double value, double floor_value) {
    return std::max(value, floor_value);
}

inline double abs_floor(double value, double floor_value) {
    return std::max(std::abs(value), floor_value);
}

} // namespace numeric_utils
