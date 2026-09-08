#pragma once

#include <string>

// An OpenRocket .ork file is a zip archive containing one XML document
// (always named "rocket.ork" inside the zip, alongside any decal images).
// This reads that XML document into memory as plain text, ready for
// ork_geometry / ork_flightdata to parse.
//
// A free function (not tied to any class -- no object/state needed, just
// input in, output out). Takes ork_path by const reference (read-only
// alias, avoids copying the string) and returns a new std::string by value
// (the caller owns the returned copy).
std::string readOrkXml(const std::string& ork_path);
