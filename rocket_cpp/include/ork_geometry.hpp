#pragma once

#include "rocket_types.hpp"
#include <string>

// Reads the fixed, physical shape of the rocket — nose cone, body tube,
// fin set — out of an OpenRocket design XML. This is everything that
// does NOT change during flight; thrust/mass/CP/CG/inertia over time
// come from ork_flightdata instead.
//
// Returns a RocketParams by value (a fresh struct, plain copy handed back
// to the caller) -- not a reference, since there's no existing object to
// point to; this function builds a new one from scratch.
RocketParams parseOrkGeometry(const std::string& xml);
