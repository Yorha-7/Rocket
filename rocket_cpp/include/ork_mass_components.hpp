#pragma once

#include <string>
#include <vector>

// One structural piece of the airframe: its mass and where its center of
// gravity sits, measured from the nose tip. This is the raw material this
// codebase uses to compute its own dry mass/CG/inertia, instead of reading
// OpenRocket's already-solved values.
struct MassComponent {
    std::string name;
    double mass_kg;
    double position_cm;  // this component's own CG, from the nose tip
};

// Walks every structural subcomponent in the .ork design (nosecone, body
// tube, fin set, parachute, shock cord, wadding, launch lug, centering
// rings, motor mount tube, engine block) and computes each one's mass from
// its own material density and geometry (bulk: density x volume, surface:
// density x area, line: density x length), or a direct <mass> tag when the
// design gives one directly (e.g. wadding).
std::vector<MassComponent> parseOrkMassComponents(const std::string& xml);
