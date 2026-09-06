#pragma once

#include "rocket_types.hpp"
#include <string>

// Reads the fixed, physical shape of the rocket — nose cone, body tube,
// fin set — out of an OpenRocket design XML. This is everything that
// does NOT change during flight; thrust/mass/CP/CG/inertia over time
// come from ork_flightdata instead.
RocketParams parseOrkGeometry(const std::string& xml);
