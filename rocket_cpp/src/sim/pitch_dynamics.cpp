#include "sim/rocket_kinematics.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

// ============================================================
// Pitch dynamics: how the rocket tips forward/backward.
//
// Two things fight for control of the pitch angle each step:
//   1. Aerodynamic force, acting at the CP, which pushes the nose
//      back toward the direction of travel whenever the body axis
//      and velocity vector disagree (real angle of attack -- see
//      buildFlightConditions -- not just absolute tilt from vertical).
//   2. Damping, which just resists whatever rotation is happening,
//      like air resistance on a spinning weathervane.
// Their combined torque, divided by how hard the rocket resists
// rotating (its moment of inertia), gives the pitch acceleration.
//
// Gravity is NOT a third contributor: a uniform gravitational field
// exerts zero net torque about a rigid body's own center of gravity,
// by definition of the CG. (computeGravityTorque is kept as a
// deliberate always-zero stub -- see below -- rather than removed, so
// the CSV/plots keep the same 3-column torque breakdown and visibly
// show that contribution as zero instead of silently disappearing.)
//
// CP/CG/inertia (MassProperties) and the normal-force slope Cn_alpha are
// both computed from the vehicle's own geometry (AerodynamicsModel,
// VehicleMassModel) -- nothing here is read from a pre-solved simulation.
// ============================================================

// Always zero: gravity acts through the CG by definition, so it can't
// exert a torque about the CG no matter where the CP sits. An earlier
// version of this modeled gravity as a pendulum restoring torque
// (mass on a rod pivoting around a fixed point) -- physically wrong for
// a body in free flight, and it dominated the (also wrong) pitch
// behavior at large launch angles.
double RocketKinematics::computeGravityTorque(const RocketState& state,
                                              const MassProperties& mp) const {
    return 0.0;
}

// Air pushing on the CP, offset from the CG, also creates a
// restoring torque — stronger at higher speed and lower altitude
// (denser air). Angle of attack is approximated by the pitch angle
// itself, which only holds for near-vertical flight with no wind.
double RocketKinematics::computeAerodynamicMoment(const RocketState& state,
                                                  const MassProperties& mp) const {
    double v = state.velocity.norm();
    if (v < 1e-6) return 0.0;

    double altitude = std::max(0.0, state.position(2));
    double rho = aero_.getDensity(altitude);

    double d = (mp.cp_location_cm - mp.cg_location_cm) / 100.0;  // CP-CG offset, m
    double A = params_.reference_area;

    FlightConditions fc = buildFlightConditions(state);
    double Cn_alpha = aero_.computeCoefficients(fc).Cn_alpha;

    double alpha_limited = std::max(-0.5, std::min(0.5, fc.alpha));

    return -0.5 * rho * v * v * Cn_alpha * alpha_limited * A * d;
}

// Resists rotation, proportional to how fast the rocket is already
// rotating — like a weathervane settling down instead of oscillating
// forever. 0.6 is OpenRocket's own empirical damping factor.
double RocketKinematics::computeDampingTorque(const RocketState& state,
                                              const MassProperties& mp) const {
    double altitude = std::max(0.0, state.position(2));
    double rho = aero_.getDensity(altitude);

    double d = (mp.cp_location_cm - mp.cg_location_cm) / 100.0;
    double A = params_.reference_area;

    double c_damp = 0.6 * 0.5 * rho * state.velocity.norm() * d * d * A;

    return -c_damp * state.angular_vel(1);
}

double RocketKinematics::computeTotalPitchTorque(const RocketState& state,
                                                 const MassProperties& mp) const {
    return computeGravityTorque(state, mp)
         + computeAerodynamicMoment(state, mp)
         + computeDampingTorque(state, mp);
}

// Newton's second law for rotation: angular acceleration = torque / inertia.
// Clamped so a bad transient (e.g. right at motor ignition) can't blow up
// the integration.
double RocketKinematics::computePitchAcceleration(double total_torque,
                                                  const MassProperties& mp) const {
    double alpha = total_torque / mp.I_yy;
    const double MAX_ALPHA = 100.0;  // rad/s^2
    return std::max(-MAX_ALPHA, std::min(MAX_ALPHA, alpha));
}

// Integrate: torque -> angular acceleration -> angular velocity -> pitch
// angle. Mass properties are recomputed from the vehicle's current mass
// (propellant burns down) and our cached Barrowman CP each step.
//
// Pitch used to hard-clamp orientation(1) to +-85 degrees for the WHOLE
// flight -- added as a numerical safety net against a bad transient
// right at ignition, per the original comment, but never actually
// scoped to just that window. In practice that meant it did something
// its own comment never intended: once a real (not transient-glitch)
// flight pushed pitch up against that wall -- e.g. sustained TVC
// saturation building a mostly-horizontal velocity, see README Staging
// Notes -- the clamp TRAPPED the vehicle there instead of letting
// computeAerodynamicMoment's restoring torque keep tracking the actual
// velocity vector past 90 degrees, the way it's always free to for yaw
// (yaw_dynamics.cpp has never clamped its angle, only wrapped it).
//
// Tried removing the clamp entirely to match yaw -- confirmed the trap
// really was the problem, but also uncovered a second, real issue:
// fully unclamped, a large enough excursion can spin up faster than the
// (already-known-weak, see Staging Notes) damping torque and
// computeAerodynamicMoment's own stall saturation (alpha capped at
// +-0.5 rad) can arrest, so the vehicle can end up genuinely tumbling
// (measured: >190 degrees, >450 deg/s) instead of settling. Fixing that
// properly means retuning the aerodynamic model, out of scope here --
// so the clamp is back, but scoped to ONLY the actual ignition window
// its comment always claimed (elapsed_time_s_ tracks that, see
// rocket_kinematics.hpp), not the entire flight. Pitch is free to wrap
// like yaw once clear of it; a trajectory that tumbles after that is a
// real (if extreme) finding about this vehicle's damping, not something
// papered over by an angle wall.
void RocketKinematics::updatePitchDynamics(RocketState& next, const RocketState& state) const {
    MassProperties mp = mass_model_.computeAt(state.mass);
    mp.cp_location_cm = cp_location_cm_;

    double total_torque = computeTotalPitchTorque(state, mp);
    double alpha = computePitchAcceleration(total_torque, mp);

    next.angular_vel(1) = state.angular_vel(1) + alpha * config_.dt;

    const double MAX_ANGULAR_VEL = 10.0;  // rad/s
    next.angular_vel(1) = std::max(-MAX_ANGULAR_VEL, std::min(MAX_ANGULAR_VEL, next.angular_vel(1)));

    next.orientation(1) = fmod(state.orientation(1) + next.angular_vel(1) * config_.dt, 2 * M_PI);

    // Ignition-transient guard, and ONLY that -- see this function's doc
    // comment. 0.1s is generous relative to how fast this vehicle's
    // motor actually ramps up (0 to ~70N in ~0.05s, per the .ork's own
    // thrust curve), so it covers the real transient without lingering
    // into the rest of the flight the way the old whole-flight clamp did.
    const double IGNITION_TRANSIENT_S = 0.1;
    if (elapsed_time_s_ <= IGNITION_TRANSIENT_S) {
        const double MAX_PITCH = 1.5;  // ~85 degrees
        next.orientation(1) = std::max(-MAX_PITCH, std::min(MAX_PITCH, next.orientation(1)));
    }
}

// Same mass properties/torque math as updatePitchDynamics, but returns
// the breakdown instead of integrating it -- for logging/plotting which
// of the three torques is actually driving the pitch at each instant.
PitchTorques RocketKinematics::computePitchTorques(const RocketState& state) const {
    MassProperties mp = mass_model_.computeAt(state.mass);
    mp.cp_location_cm = cp_location_cm_;

    PitchTorques torques{};
    torques.gravity = computeGravityTorque(state, mp);
    torques.aerodynamic = computeAerodynamicMoment(state, mp);
    torques.damping = computeDampingTorque(state, mp);
    return torques;
}
