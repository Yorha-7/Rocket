#include "ork/ork_mass_components.hpp"
#include <tinyxml2.h>
#include <stdexcept>
#include <cmath>
#include <cstring>

using tinyxml2::XMLElement;
using tinyxml2::XMLDocument;

namespace {

// ##### XML helpers #####
// Goal: small, reusable "find this tag, read its value" pieces the
// per-component sections below all build on.

const XMLElement* findFirst(const XMLElement* root, const char* name) {
    if (!root) return nullptr;
    if (std::strcmp(root->Name(), name) == 0) return root;
    for (const XMLElement* child = root->FirstChildElement(); child; child = child->NextSiblingElement()) {
        const XMLElement* found = findFirst(child, name);
        if (found) return found;
    }
    return nullptr;
}

double childDouble(const XMLElement* parent, const char* tag, double fallback = 0.0) {
    if (!parent) return fallback;
    const XMLElement* child = parent->FirstChildElement(tag);
    if (!child || !child->GetText()) return fallback;
    try {
        return std::stod(child->GetText());
    } catch (...) {
        return fallback;
    }
}

// Goal: a component's own axial extent, for placing its CG at the
// center of that extent rather than right on its reference edge. Packed
// items (parachute, shock cord, wadding) report <packedlength>; solid
// parts report <length>.
double ownLength(const XMLElement* elem) {
    double packed = childDouble(elem, "packedlength", -1.0);
    if (packed >= 0.0) return packed;
    return childDouble(elem, "length", 0.0);
}

// Goal: read a component's material density, whichever kind of material
// tag it uses -- bulk (density x volume), surface (e.g. parachute
// canopy), or line (e.g. shock cord).
double materialDensity(const XMLElement* parent, const char* tag = "material") {
    const XMLElement* mat = parent->FirstChildElement(tag);
    if (!mat || !mat->Attribute("density")) return 0.0;
    return std::stod(mat->Attribute("density"));
}

// Goal: work out a component's CG position (cm from the nose tip) from
// its <position type="top|bottom|middle"> relative to its immediate
// parent. Validated against this design: "top" measures forward-to-aft
// from the parent's start; "bottom" measures from the parent's end
// (offset can be negative, moving the component forward, into the
// parent); "middle" centers on the parent's own midpoint.
double resolveCg(double parent_start_cm, double parent_length_cm,
                 const XMLElement* elem, double own_length_m) {
    const XMLElement* pos = elem->FirstChildElement("position");
    double half_cm = own_length_m * 100.0 / 2.0;
    if (!pos) return parent_start_cm + half_cm;

    std::string type = pos->Attribute("type") ? pos->Attribute("type") : "top";
    double offset_cm = pos->GetText() ? std::stod(pos->GetText()) * 100.0 : 0.0;

    if (type == "bottom") {
        double edge = parent_start_cm + parent_length_cm + offset_cm;
        return edge - half_cm;
    }
    if (type == "middle") {
        return parent_start_cm + parent_length_cm / 2.0 + offset_cm;
    }
    if (type == "absolute") {
        return offset_cm;
    }
    // "top" (default)
    double edge = parent_start_cm + offset_cm;
    return edge + half_cm;
}

}  // namespace

// ##### parseOrkMassComponents() #####
// Goal: go through the airframe piece by piece, working out each one's
// mass from its own shape/density and where its CG lands.
std::vector<MassComponent> parseOrkMassComponents(const std::string& xml) {
    XMLDocument doc;
    if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Failed to parse .ork XML");
    }

    const XMLElement* rocket = findFirst(doc.RootElement(), "rocket");
    const XMLElement* nosecone = findFirst(rocket, "nosecone");
    const XMLElement* bodytube = findFirst(rocket, "bodytube");
    if (!nosecone || !bodytube) {
        throw std::runtime_error("Could not find nosecone/bodytube in the .ork file");
    }

    std::vector<MassComponent> components;

    // ##### Nose cone: thin conical shell #####
    double nose_len_m = childDouble(nosecone, "length");
    double nose_r_m = childDouble(nosecone, "aftradius");
    double nose_thickness_m = childDouble(nosecone, "thickness");
    double nose_density = materialDensity(nosecone);
    double nose_slant_m = std::sqrt(nose_len_m * nose_len_m + nose_r_m * nose_r_m);
    double nose_lateral_area_m2 = M_PI * nose_r_m * nose_slant_m;
    double nose_mass_kg = nose_lateral_area_m2 * nose_thickness_m * nose_density;
    // A thin conical shell's centroid sits 2/3 of the way from the tip to the base.
    components.push_back({"nosecone", nose_mass_kg, (2.0 / 3.0) * nose_len_m * 100.0});

    // ##### Body tube: annulus #####
    double body_start_cm = nose_len_m * 100.0;
    double body_len_m = childDouble(bodytube, "length");
    double body_r_out_m = 0.0125;  // fallback; overwritten below from the "auto r" radius text
    {
        const XMLElement* r_elem = bodytube->FirstChildElement("radius");
        if (r_elem && r_elem->GetText()) {
            std::string text(r_elem->GetText());
            size_t space = text.find_last_of(" \t");
            body_r_out_m = std::stod(space != std::string::npos ? text.substr(space + 1) : text);
        }
    }
    double body_thickness_m = childDouble(bodytube, "thickness");
    double body_r_in_m = body_r_out_m - body_thickness_m;
    double body_density = materialDensity(bodytube);
    double body_mass_kg = M_PI * (body_r_out_m * body_r_out_m - body_r_in_m * body_r_in_m) *
                          body_len_m * body_density;
    components.push_back({"bodytube", body_mass_kg, body_start_cm + body_len_m * 100.0 / 2.0});

    // Goal: everything below lives one level deeper, under bodytube's
    // own <subcomponents>, not as a direct child of <bodytube>.
    const XMLElement* body_sub = bodytube->FirstChildElement("subcomponents");
    if (!body_sub) return components;

    // ##### Parachute: surface-density canopy + line-density shroud lines #####
    if (const XMLElement* chute = body_sub->FirstChildElement("parachute")) {
        double diameter_m = childDouble(chute, "diameter");
        double canopy_area_m2 = M_PI * (diameter_m / 2.0) * (diameter_m / 2.0);
        double canopy_density = materialDensity(chute);
        double canopy_mass_kg = canopy_area_m2 * canopy_density;

        double line_count = childDouble(chute, "linecount");
        double line_length_m = childDouble(chute, "linelength");
        double line_density = materialDensity(chute, "linematerial");
        double lines_mass_kg = line_count * line_length_m * line_density;

        double cg = resolveCg(body_start_cm, body_len_m * 100.0, chute, ownLength(chute));
        components.push_back({"parachute", canopy_mass_kg + lines_mass_kg, cg});
    }

    // ##### Shock cord: line density #####
    if (const XMLElement* cord = body_sub->FirstChildElement("shockcord")) {
        double cord_len_m = childDouble(cord, "cordlength");
        double cord_density = materialDensity(cord);
        double cg = resolveCg(body_start_cm, body_len_m * 100.0, cord, ownLength(cord));
        components.push_back({"shockcord", cord_len_m * cord_density, cg});
    }

    // ##### Generic mass components (e.g. wadding): explicit <mass> #####
    for (const XMLElement* mc = body_sub->FirstChildElement("masscomponent"); mc;
         mc = mc->NextSiblingElement("masscomponent")) {
        double mass_kg = childDouble(mc, "mass");
        double cg = resolveCg(body_start_cm, body_len_m * 100.0, mc, ownLength(mc));
        components.push_back({"masscomponent", mass_kg, cg});
    }

    // ##### Launch lug: thin cylindrical shell #####
    if (const XMLElement* lug = body_sub->FirstChildElement("launchlug")) {
        double r_out = childDouble(lug, "radius");
        double thickness = childDouble(lug, "thickness");
        double r_in = r_out - thickness;
        double len_m = childDouble(lug, "length");
        double density = materialDensity(lug);
        double mass_kg = M_PI * (r_out * r_out - r_in * r_in) * len_m * density;
        double cg = resolveCg(body_start_cm, body_len_m * 100.0, lug, len_m);
        components.push_back({"launchlug", mass_kg, cg});
    }

    // ##### Fin set: flat trapezoidal plates (rectangle when root==tip) #####
    if (const XMLElement* fins = body_sub->FirstChildElement("trapezoidfinset")) {
        double fin_count = childDouble(fins, "fincount", 3);
        double root_m = childDouble(fins, "rootchord");
        double tip_m = childDouble(fins, "tipchord");
        double span_m = childDouble(fins, "height");
        double sweep_m = childDouble(fins, "sweeplength");
        double thickness_m = childDouble(fins, "thickness");
        double density = materialDensity(fins);

        double area_per_fin_m2 = 0.5 * (root_m + tip_m) * span_m;
        double mass_kg = fin_count * area_per_fin_m2 * thickness_m * density;

        double offset_cm = 0.0;
        if (const XMLElement* pos = fins->FirstChildElement("position")) {
            if (pos->GetText()) offset_cm = std::stod(pos->GetText()) * 100.0;
        }
        double root_le_cm = body_start_cm + body_len_m * 100.0 + offset_cm - root_m * 100.0;
        // Standard trapezoid area centroid, measured aft from the root leading edge.
        double centroid_from_root_le_m =
            (sweep_m * (root_m + 2.0 * tip_m)) / (3.0 * (root_m + tip_m)) +
            (1.0 / 6.0) * (root_m + tip_m - (root_m * tip_m) / (root_m + tip_m));
        components.push_back({"finset", mass_kg, root_le_cm + centroid_from_root_le_m * 100.0});
    }

    // ##### Centering rings: annulus #####
    // Goal: "auto" radii resolve against the body tube's inner radius
    // and the motor mount tube's outer radius.
    const XMLElement* innertube = body_sub->FirstChildElement("innertube");
    double mount_r_out_m = innertube ? childDouble(innertube, "outerradius") : 0.0;
    for (const XMLElement* ring = body_sub->FirstChildElement("centeringring"); ring;
         ring = ring->NextSiblingElement("centeringring")) {
        double len_m = childDouble(ring, "length");
        double density = materialDensity(ring);
        double r_out = body_r_in_m;
        double r_in = mount_r_out_m;
        double mass_kg = M_PI * (r_out * r_out - r_in * r_in) * len_m * density;
        double cg = resolveCg(body_start_cm, body_len_m * 100.0, ring, len_m);
        components.push_back({"centeringring", mass_kg, cg});
    }

    // ##### Motor mount tube + engine block #####
    if (innertube) {
        double len_m = childDouble(innertube, "length");
        double thickness_m = childDouble(innertube, "thickness");
        double r_in_m = mount_r_out_m - thickness_m;
        double density = materialDensity(innertube);
        double mass_kg = M_PI * (mount_r_out_m * mount_r_out_m - r_in_m * r_in_m) * len_m * density;
        double cg = resolveCg(body_start_cm, body_len_m * 100.0, innertube, len_m);
        components.push_back({"innertube", mass_kg, cg});

        double mount_start_cm = cg - len_m * 100.0 / 2.0;  // recover the tube's own start
        const XMLElement* innertube_sub = innertube->FirstChildElement("subcomponents");
        if (const XMLElement* block = innertube_sub ? innertube_sub->FirstChildElement("engineblock") : nullptr) {
            double block_len_m = childDouble(block, "length");
            double block_thickness_m = childDouble(block, "thickness");
            double block_r_out_m = mount_r_out_m - thickness_m;  // fits inside the mount tube
            double block_r_in_m = block_r_out_m - block_thickness_m;
            double block_density = materialDensity(block);
            double block_mass_kg = M_PI * (block_r_out_m * block_r_out_m - block_r_in_m * block_r_in_m) *
                                   block_len_m * block_density;
            double block_cg = resolveCg(mount_start_cm, len_m * 100.0, block, block_len_m);
            components.push_back({"engineblock", block_mass_kg, block_cg});
        }
    }

    return components;
}
