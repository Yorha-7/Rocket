#pragma once

#include "sim/rocket_types.hpp"
#include <string>

// ##### parseOrkGeometry() #####
// Goal: read the rocket's fixed, physical SHAPE -- nose cone, body tube,
// fin set -- straight out of the OpenRocket design XML. This is
// everything that does NOT change during flight; thrust/mass/CP/CG/
// inertia over time come from ork_flightdata instead. Returns a fresh
// RocketParams built from scratch, not a reference to anything existing.
RocketParams parseOrkGeometry(const std::string& xml);
