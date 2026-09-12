#pragma once

#include <string>

// ##### readOrkXml() #####
// Goal: an .ork file is really a zip archive containing one XML document
// (always named "rocket.ork" inside the zip, alongside any decal
// images). This pulls that XML out into a plain string, ready for
// ork_geometry / ork_flightdata / ork_mass_components to parse -- this
// function doesn't interpret any of the rocket's actual data itself.
std::string readOrkXml(const std::string& ork_path);
