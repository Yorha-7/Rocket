#include "ork/ork_geometry.hpp"
#include <tinyxml2.h>
#include <stdexcept>
#include <cmath>
#include <cstring>

using tinyxml2::XMLElement;
using tinyxml2::XMLDocument;

namespace {

// ##### XML helpers #####
// Goal: small, reusable pieces of "find this tag, read its value" logic
// so the actual geometry-reading code below doesn't repeat itself.

// Goal: find the first element named `name` anywhere under `root`
// (depth-first), since component nesting can vary between .ork designs
// -- search by name instead of hardcoding a fixed path.
const XMLElement* findFirst(const XMLElement* root, const char* name) {
    if (!root) return nullptr;
    if (std::strcmp(root->Name(), name) == 0) return root;
    for (const XMLElement* child = root->FirstChildElement(); child; child = child->NextSiblingElement()) {
        const XMLElement* found = findFirst(child, name);
        if (found) return found;
    }
    return nullptr;
}

// Goal: read a direct child's text as a double, or fall back if it's
// missing. Also handles OpenRocket's "auto <value>" radius syntax by
// taking the last whitespace-separated token.
double childDouble(const XMLElement* parent, const char* tag, double fallback = 0.0) {
    if (!parent) return fallback;
    const XMLElement* child = parent->FirstChildElement(tag);
    if (!child || !child->GetText()) return fallback;

    std::string text(child->GetText());
    size_t space = text.find_last_of(" \t");
    if (space != std::string::npos) text = text.substr(space + 1);

    try {
        return std::stod(text);
    } catch (...) {
        return fallback;
    }
}

std::string childText(const XMLElement* parent, const char* tag, const std::string& fallback) {
    if (!parent) return fallback;
    const XMLElement* child = parent->FirstChildElement(tag);
    if (!child || !child->GetText()) return fallback;
    return child->GetText();
}

// Goal: map OpenRocket's nose-shape names onto this codebase's shape
// codes -- unrecognized shapes fall back to ogive, the most common case.
int noseShapeCode(const std::string& shape) {
    if (shape == "conical") return 0;
    if (shape == "ogive") return 1;
    if (shape == "ellipsoid") return 2;
    if (shape == "parabolic" || shape == "power" || shape == "haack") return 3;
    return 1;
}

// Goal: approximate RMS roughness (m) for OpenRocket's named surface
// finishes -- typical published figures, not critical precision, since
// only the staged aerodynamics.hpp model reads this today.
double roughnessForFinish(const std::string& finish) {
    if (finish == "polished") return 2.0e-6;
    if (finish == "smooth") return 6.0e-6;
    if (finish == "unfinished" || finish == "rough") return 150.0e-6;
    return 20.0e-6;  // "normal", or unspecified
}

}  // namespace

// ##### parseOrkGeometry() #####
// Goal: locate the nose cone, body tube, and (optional) fin set in the
// XML, then read each one's own shape/size fields straight into a
// RocketParams -- no aerodynamic computation happens here, just reading
// numbers off the page.
RocketParams parseOrkGeometry(const std::string& xml) {
    XMLDocument doc;
    if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Failed to parse .ork XML");
    }

    const XMLElement* rocket = findFirst(doc.RootElement(), "rocket");
    const XMLElement* nosecone = findFirst(rocket, "nosecone");
    const XMLElement* bodytube = findFirst(rocket, "bodytube");
    const XMLElement* finset = findFirst(rocket, "trapezoidfinset");

    if (!nosecone || !bodytube) {
        throw std::runtime_error("Could not find a nosecone/bodytube in the .ork file");
    }

    RocketParams params{};

    params.nose_length = childDouble(nosecone, "length");
    params.nose_shape = noseShapeCode(childText(nosecone, "shape", "ogive"));
    params.nose_radius = 0.0;  // no blunted-tip radius unless the design specifies one

    params.body_length = childDouble(bodytube, "length");
    double body_radius = childDouble(bodytube, "radius", childDouble(nosecone, "aftradius"));
    params.body_diameter = 2.0 * body_radius;
    params.base_diameter = params.body_diameter;  // no boat-tail/transition in this design
    params.boat_tail_length = 0.0;

    params.surface_roughness = roughnessForFinish(childText(bodytube, "finish", "normal"));
    params.reference_area = M_PI * body_radius * body_radius;

    if (finset) {
        params.fin_count = static_cast<int>(childDouble(finset, "fincount", 3));
        params.fin_root_chord = childDouble(finset, "rootchord");
        params.fin_tip_chord = childDouble(finset, "tipchord");
        params.fin_span = childDouble(finset, "height");
        params.fin_sweep = childDouble(finset, "sweeplength");
        params.fin_thickness = childDouble(finset, "thickness");
        params.fin_cant = childDouble(finset, "cant");

        // Goal: fin position is type="bottom" -- an offset from the
        // tube's aft end, ADDED (offset 0 = flush with the aft end,
        // negative moves the fin forward, into the tube). Validated
        // against this design's own centering-ring/motor-mount offsets
        // in ork_mass_components.cpp.
        const XMLElement* position = finset->FirstChildElement("position");
        double offset = (position && position->GetText()) ? std::stod(position->GetText()) : 0.0;
        double tube_aft_from_nose = params.nose_length + params.body_length;
        params.fin_root_le_position = tube_aft_from_nose + offset - params.fin_root_chord;
    } else {
        params.fin_count = 0;
        params.fin_root_chord = params.fin_tip_chord = params.fin_span = 0.0;
        params.fin_sweep = params.fin_thickness = params.fin_cant = 0.0;
        params.fin_root_le_position = 0.0;
    }

    // Goal: leave these informational -- the sim reads the real per-step
    // thrust curve from FlightData instead of using these fields.
    params.thrust_duration = 0.0;
    params.max_thrust = 0.0;

    return params;
}
