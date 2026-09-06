#pragma once

#include "rocket_types.hpp"
#include "ork_mass_components.hpp"
#include <vector>

// Computes the vehicle's own mass/CG/inertia from its structural
// components instead of reading OpenRocket's solved values. Dry
// (motor-less) mass properties are fixed at construction; computeAt()
// combines them with the motor's current total mass (from FlightData --
// the one thing we still can't compute ourselves without a motor database)
// to get mass properties at any instant.
class VehicleMassModel {
public:
    VehicleMassModel(const std::vector<MassComponent>& components,
                     double body_diameter_m, double body_length_m);

    // total_mass_kg: current total vehicle mass (structure + motor), from
    // FlightData. cp_location_cm in the result is left at 0 -- that's
    // AerodynamicsModel's job, not this class's.
    MassProperties computeAt(double total_mass_kg) const;

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
