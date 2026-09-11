#pragma once

#include "sim/rocket_types.hpp"
#include "ork/ork_mass_components.hpp"
#include <vector>

// Computes the vehicle's own mass/CG/inertia from its structural
// components instead of reading OpenRocket's solved values. Dry
// (motor-less) mass properties are fixed at construction; computeAt()
// combines them with the motor's current total mass (from FlightData --
// the one thing we still can't compute ourselves without a motor database)
// to get mass properties at any instant.
class VehicleMassModel {
public:
    // Constructor takes 3 arguments, so it's never a candidate for the
    // silent single-argument conversion "explicit" guards against --
    // no explicit needed here.
    VehicleMassModel(const std::vector<MassComponent>& components,
                     double body_diameter_m, double body_length_m);

    // total_mass_kg: current total vehicle mass (structure + motor), from
    // FlightData. cp_location_cm in the result is left at 0 -- that's
    // AerodynamicsModel's job, not this class's.
    MassProperties computeAt(double total_mass_kg) const;

    // Getters (read-only access to a private value). Written directly in
    // the header like this, they're implicitly inline -- the compiler
    // substitutes the one-line body at each call site instead of a real
    // function call, since it's cheap enough not to need a .cpp definition.
    double dryMassKg() const { return dry_mass_kg_; }
    double dryCgCm() const { return dry_cg_cm_; }

private:  // hidden from outside code -- can only change via the constructor above
    double dry_mass_kg_;
    double dry_cg_cm_;
    double dry_I_yy_;        // about dry_cg_cm_
    double motor_position_cm_;
    double body_radius_m_;   // for the coarse I_xx estimate
    double motor_radius_m_;
};
