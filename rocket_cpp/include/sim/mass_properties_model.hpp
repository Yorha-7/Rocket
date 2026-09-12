#pragma once

#include "sim/rocket_types.hpp"
#include "ork/ork_mass_components.hpp"
#include <vector>

// ##### VehicleMassModel #####
// Goal: work out the vehicle's own mass, CG, and inertia from its
// structural components, instead of trusting OpenRocket's solved
// values. The dry (motor-less) mass properties are fixed once at
// construction; computeAt() then combines them with however much motor
// mass is currently left (from FlightData -- the one thing we still
// can't derive ourselves without a motor database) to get mass
// properties at any single instant.
class VehicleMassModel {
public:
    // 3 arguments -- never a candidate for the single-argument implicit
    // conversion "explicit" guards against, so no explicit needed here.
    VehicleMassModel(const std::vector<MassComponent>& components,
                     double body_diameter_m, double body_length_m);

    // Goal: mass properties at one instant, given the vehicle's current
    // TOTAL mass (structure + whatever propellant is left), from
    // FlightData. cp_location_cm comes back as 0 -- that's
    // AerodynamicsModel's job, not this class's.
    MassProperties computeAt(double total_mass_kg) const;

    // Goal: expose the fixed dry-structure numbers computed once at
    // construction, for logging/sanity-checking. Defined right here in
    // the header (implicitly inline) since each is a one-line
    // pass-through, too cheap to need a real out-of-line function.
    double dryMassKg() const { return dry_mass_kg_; }
    double dryCgCm() const { return dry_cg_cm_; }

private:
    double dry_mass_kg_;
    double dry_cg_cm_;
    double dry_I_yy_;        // about dry_cg_cm_
    double motor_position_cm_;
    double body_radius_m_;   // for the coarse I_xx estimate
    double motor_radius_m_;
};
