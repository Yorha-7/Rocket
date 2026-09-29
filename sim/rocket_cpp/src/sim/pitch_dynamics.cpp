#include "sim/rocket_kinematics.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

// ##### Pitch dynamics, the big picture #####
// Goal: work out how the rocket tips forward/backward each step. Two
// things fight over the pitch angle: aerodynamic force at the CP trying
// to swing the nose back into the airflow (weathercocking), and damping
// resisting whatever rotation is already happening. Their combined
// torque, divided by how hard the rocket resists spinning (I_yy), gives
// pitch acceleration -- straight Newton's second law for rotation.
// Gravity is NOT a third contributor: a uniform field can't exert any
// net torque about a rigid body's own center of gravity, by definition
// of the CG.

// ##### computeGravityTorque() #####
// Goal: always return zero, deliberately -- see the note above. Kept as
// a real (empty) function rather than deleted so the CSV/plots keep the
// same 3-column torque breakdown and visibly show "zero," instead of
// that column silently disappearing.
double RocketKinematics::computeGravityTorque(const RocketState& state,
                                              const MassProperties& mp) const {
    return 0.0;
}

// ##### computeAerodynamicMoment() #####
// Goal: work out the restoring torque from air pushing on the CP,
// offset from the CG -- stronger at higher speed and denser (lower)
// air, and proportional to how far the nose is actually off the airflow
// (real angle of attack, not just tilt from vertical). Capped at ~28.6
// degrees of angle of attack (alpha_limited) since the underlying
// Cn_alpha model is a straight-line approximation that stops being
// trustworthy at extreme angles (real air starts separating/stalling).
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

// ##### computeDampingTorque() #####
// Goal: resist whatever rotation is currently happening, proportional to
// how fast it's spinning -- like a weathervane settling down instead of
// swinging forever. 0.6 is OpenRocket's own empirical damping factor for
// this shape of formula. (This coefficient has been measured elsewhere
// this session as noticeably weak -- see README Staging Notes -- worth
// knowing if a trajectory oscillates for longer than expected.)
double RocketKinematics::computeDampingTorque(const RocketState& state,
                                              const MassProperties& mp) const {
    double altitude = std::max(0.0, state.position(2));
    double rho = aero_.getDensity(altitude);

    double d = (mp.cp_location_cm - mp.cg_location_cm) / 100.0;
    double A = params_.reference_area;

    double c_damp = 0.6 * 0.5 * rho * state.velocity.norm() * d * d * A;

    return -c_damp * state.angular_vel(1);
}

// Goal: add the three torque sources into the one number the integrator
// actually uses.
double RocketKinematics::computeTotalPitchTorque(const RocketState& state,
                                                 const MassProperties& mp) const {
    return computeGravityTorque(state, mp)
         + computeAerodynamicMoment(state, mp)
         + computeDampingTorque(state, mp);
}

// ##### computePitchAcceleration() #####
// Goal: Newton's second law for rotation -- angular acceleration =
// torque / inertia. Clamped so a bad transient (e.g. right at motor
// ignition, before the vehicle has picked up real airspeed) can't blow
// the integration up to nonsense.
double RocketKinematics::computePitchAcceleration(double total_torque,
                                                  const MassProperties& mp) const {
    double alpha = total_torque / mp.I_yy;
    const double MAX_ALPHA = 100.0;  // rad/s^2
    return std::max(-MAX_ALPHA, std::min(MAX_ALPHA, alpha));
}

// ##### updatePitchDynamics() #####
// Goal: integrate torque -> angular acceleration -> angular rate ->
// pitch angle, one dt at a time. Mass properties are recomputed from the
// vehicle's current mass (propellant burns down) each step.
//
// Pitch angle wraps (fmod) rather than clamps, same as yaw -- neither
// axis is physically bounded to a narrow range; a vehicle CAN legitimately
// end up nose-down or flying past 90 degrees of tilt. An angle clamp here
// used to run for the WHOLE flight (not just ignition) and ended up
// trapping the vehicle at its wall instead of letting the real restoring
// torque above keep tracking wherever the velocity vector actually went
// -- see README Staging Notes for the full writeup and the follow-on
// finding it led to (a real, separate bug in how drag was computed,
// also documented there).
void RocketKinematics::updatePitchDynamics(RocketState& next, const RocketState& state) const {
    MassProperties mp = mass_model_.computeAt(state.mass);
    mp.cp_location_cm = cp_location_cm_;

    double total_torque = computeTotalPitchTorque(state, mp);
    double alpha = computePitchAcceleration(total_torque, mp);

    next.angular_vel(1) = state.angular_vel(1) + alpha * config_.dt;

    const double MAX_ANGULAR_VEL = 10.0;  // rad/s
    next.angular_vel(1) = std::max(-MAX_ANGULAR_VEL, std::min(MAX_ANGULAR_VEL, next.angular_vel(1)));

    next.orientation(1) = fmod(state.orientation(1) + next.angular_vel(1) * config_.dt, 2 * M_PI);

    // Goal: ONLY guard the true ignition transient (first 0.1s, generous
    // against the motor's real ~0.05s ramp-up per the .ork's own thrust
    // curve) -- not the whole flight. Past that window, pitch is free to
    // wrap like yaw always has.
    const double IGNITION_TRANSIENT_S = 0.1;
    if (elapsed_time_s_ <= IGNITION_TRANSIENT_S) {
        const double MAX_PITCH = 1.5;  // ~85 degrees
        next.orientation(1) = std::max(-MAX_PITCH, std::min(MAX_PITCH, next.orientation(1)));
    }
}

// ##### computePitchTorques() #####
// Goal: same math as updatePitchDynamics, but return the three torques
// broken out instead of integrating them -- for logging/plotting which
// one is actually driving pitch at a given instant.
PitchTorques RocketKinematics::computePitchTorques(const RocketState& state) const {
    MassProperties mp = mass_model_.computeAt(state.mass);
    mp.cp_location_cm = cp_location_cm_;

    PitchTorques torques{};
    torques.gravity = computeGravityTorque(state, mp);
    torques.aerodynamic = computeAerodynamicMoment(state, mp);
    torques.damping = computeDampingTorque(state, mp);
    return torques;
}
