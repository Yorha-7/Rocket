#pragma once

#include "gnc/navigation.hpp"
#include <string>

namespace interception {
constexpr double MIN_SEGMENT_SQUARED_M2 = 1e-18;
constexpr double PI = 3.14159265358979323846;
constexpr double DEFAULT_RADIUS_M = 500.0;
constexpr int DEFAULT_SHELLS = 16;
constexpr int DEFAULT_DIRECTIONS = 96;
constexpr double MIN_TARGET_ALTITUDE_M = navigation::Navigation::MIN_TARGET_ALTITUDE_M;

struct SweepOptions {
    double radius_m = DEFAULT_RADIUS_M;
    int shells = DEFAULT_SHELLS;
    int directions = DEFAULT_DIRECTIONS;
    double init_tilt_deg = 0.0;
    double init_yaw_deg = 10.0;
    bool preview = true;
    bool terminal = false;
    bool output_explicit = false;
    std::string output_path;
};

constexpr double REFERENCE_TARGET_CLEARANCE_M = 10.0;
constexpr int PROGRESS_INTERVAL = 25;
constexpr double SUCCESS_NEAR_M = 5.0;
constexpr double SUCCESS_MEDIUM_M = 10.0;
constexpr double SUCCESS_FAR_M = 25.0;
}  // namespace interception
