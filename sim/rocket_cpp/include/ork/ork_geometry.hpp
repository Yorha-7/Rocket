#pragma once

#include "sim/rocket_types.hpp"
#include <string>

namespace ork_geometry_settings {
inline constexpr double POLISHED_ROUGHNESS_M = 2.0e-6;
inline constexpr double SMOOTH_ROUGHNESS_M = 6.0e-6;
inline constexpr double UNFINISHED_ROUGHNESS_M = 150.0e-6;
inline constexpr double NORMAL_ROUGHNESS_M = 20.0e-6;
}

// ##### parseOrkGeometry() #####
// Goal: read the rocket's fixed, physical SHAPE -- nose cone, body tube,
// fin set -- straight out of the OpenRocket design XML. This is
// everything that does NOT change during flight; thrust/mass/CP/CG/
// inertia over time come from ork_flightdata instead. Returns a fresh
// RocketParams built from scratch, not a reference to anything existing.
RocketParams parseOrkGeometry(const std::string& xml);
