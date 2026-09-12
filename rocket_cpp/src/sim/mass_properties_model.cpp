#include "sim/mass_properties_model.hpp"
#include <algorithm>

namespace {
constexpr double kMotorRadiusFallback_m = 0.009;
}

// ##### Constructor: fix the dry-structure numbers once #####
// Goal: sum every structural component's mass into one total, find their
// combined center of gravity (mass-weighted average position), and find
// the dry structure's own moment of inertia about that CG -- all of this
// never changes mid-flight, so it's worth computing exactly once here
// instead of every step.
VehicleMassModel::VehicleMassModel(const std::vector<MassComponent>& components,
                                   double body_diameter_m, double body_length_m)
    : dry_mass_kg_(0.0), dry_cg_cm_(0.0), dry_I_yy_(0.0),
      motor_position_cm_(0.0), body_radius_m_(body_diameter_m / 2.0),
      motor_radius_m_(kMotorRadiusFallback_m) {

    for (const auto& c : components) {
        dry_mass_kg_ += c.mass_kg;
        dry_cg_cm_ += c.mass_kg * c.position_cm;
        if (c.name == "innertube") motor_position_cm_ = c.position_cm;
    }
    if (dry_mass_kg_ > 1e-9) dry_cg_cm_ /= dry_mass_kg_;
    if (motor_position_cm_ == 0.0) motor_position_cm_ = dry_cg_cm_;

    // Goal: parallel-axis theorem, treating each component as a point
    // mass at its own position -- plus one correction for the body tube,
    // since it spans a large fraction of the vehicle's length and a
    // point-mass approximation would understate its real contribution to
    // pitch inertia (a rod's own inertia about its center is nonzero).
    for (const auto& c : components) {
        double d_m = (c.position_cm - dry_cg_cm_) / 100.0;
        dry_I_yy_ += c.mass_kg * d_m * d_m;
        if (c.name == "bodytube") {
            dry_I_yy_ += c.mass_kg * (body_length_m * body_length_m) / 12.0;
        }
    }
}

// ##### computeAt() #####
// Goal: combine the fixed dry-structure numbers above with however much
// motor mass is left RIGHT NOW to get this instant's actual mass, CG,
// and inertia -- called every physics step, since propellant burns down
// continuously through the flight.
MassProperties VehicleMassModel::computeAt(double total_mass_kg) const {
    double motor_mass_kg = std::max(0.0, total_mass_kg - dry_mass_kg_);
    double combined_mass_kg = dry_mass_kg_ + motor_mass_kg;

    double combined_cg_cm = combined_mass_kg > 1e-9
        ? (dry_mass_kg_ * dry_cg_cm_ + motor_mass_kg * motor_position_cm_) / combined_mass_kg
        : dry_cg_cm_;

    // Goal: shift the dry structure's own inertia onto the NEW combined
    // CG (it moves as propellant burns off), then add the motor in as
    // its own point mass at its fixed position.
    double d_dry_m = (dry_cg_cm_ - combined_cg_cm) / 100.0;
    double d_motor_m = (motor_position_cm_ - combined_cg_cm) / 100.0;
    double I_yy = (dry_I_yy_ + dry_mass_kg_ * d_dry_m * d_dry_m) +
                 motor_mass_kg * d_motor_m * d_motor_m;

    // Goal: a coarse roll-axis inertia estimate -- thin shell for the
    // dry structure, solid cylinder for the motor. Nothing in the sim
    // actually reads I_xx yet (no roll torque model exists), so this
    // doesn't need to be precise until that's built.
    double I_xx = dry_mass_kg_ * body_radius_m_ * body_radius_m_ +
                 0.5 * motor_mass_kg * motor_radius_m_ * motor_radius_m_;

    MassProperties mp{};
    mp.cp_location_cm = 0.0;
    mp.cg_location_cm = combined_cg_cm;
    mp.I_xx = I_xx;
    mp.I_yy = I_yy;
    mp.I_zz = I_yy;
    return mp;
}
