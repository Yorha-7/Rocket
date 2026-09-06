#pragma once

#include <string>

// An OpenRocket .ork file is a zip archive containing one XML document
// (always named "rocket.ork" inside the zip, alongside any decal images).
// This reads that XML document into memory as plain text, ready for
// ork_geometry / ork_flightdata to parse.
std::string readOrkXml(const std::string& ork_path);
