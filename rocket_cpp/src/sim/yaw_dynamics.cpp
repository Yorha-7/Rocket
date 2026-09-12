#include "sim/rocket_kinematics.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

// ============================================================
// Yaw dynamics: how the rocket swings side to side -- the exact mirror of
// pitch_dynamics.cpp, one plane over (body Y-Z instead of body X-Z), using
// sideslip (beta) instead of angle of attack (alpha), and the same
// Cn_alpha/CP/CG/I_zz an axisymmetric body already shares with pitch
// (I_zz = I_yy, see MassProperties). Before this existed, yaw only ever
// held whatever init_yaw set it to -- real weathercocking now gives it
// the same self-correcting behavior pitch has always had, rather than
// leaving y-axis steering (e.g. TVC/Navigation) with no vehicle-attitude
// assist at all.
//
// Both angles wrap with fmod rather than clamp -- neither pitch nor yaw
// is physically bounded to a narrow range (a vehicle can legitimately
// fly nose-down, inverted, or through any azimuth); pitch used to hard-
// clamp near +-90 degrees, which turned out to trap it there instead of
// letting the same restoring torque this file has always used settle it
// the way yaw does -- see pitch_dynamics.cpp's updatePitchDynamics().
// ============================================================

// Always zero, same reasoning as computeGravityTorque: no net torque
// about the CG from a uniform field, regardless of which axis.
double RocketKinematics::computeYawGravityTorque(const RocketState& state,
                                                 const MassProperties& mp) const {
    return 0.0;
}

// Air pushing on the CP, offset from the CG, restoring the nose toward
// the direction of travel in the yaw plane -- same physics as pitch's
// aero moment, using sideslip (beta) instead of angle of attack (alpha).
double RocketKinematics::computeYawAeroMoment(const RocketState& state,
                                              const MassProperties& mp) const {
    double v = state.velocity.norm();
    if (v < 1e-6) return 0.0;

    double altitude = std::max(0.0, state.position(2));
    double rho = aero_.getDensity(altitude);

    double d = (mp.cp_location_cm - mp.cg_location_cm) / 100.0;
    double A = params_.reference_area;

    FlightConditions fc = buildFlightConditions(state);
    double Cn_alpha = aero_.computeCoefficients(fc).Cn_alpha;

    double beta_limited = std::max(-0.5, std::min(0.5, fc.beta));

    return -0.5 * rho * v * v * Cn_alpha * beta_limited * A * d;
}

// Resists yaw rotation, proportional to yaw rate -- same weathervane
// damping as pitch, mirrored to the yaw axis.
double RocketKinematics::computeYawDampingTorque(const RocketState& state,
                                                 const MassProperties& mp) const {
    double altitude = std::max(0.0, state.position(2));
    double rho = aero_.getDensity(altitude);

    double d = (mp.cp_location_cm - mp.cg_location_cm) / 100.0;
    double A = params_.reference_area;

    double c_damp = 0.6 * 0.5 * rho * state.velocity.norm() * d * d * A;

    return -c_damp * state.angular_vel(2);
}

double RocketKinematics::computeTotalYawTorque(const RocketState& state,
                                               const MassProperties& mp) const {
    return computeYawGravityTorque(state, mp)
         + computeYawAeroMoment(state, mp)
         + computeYawDampingTorque(state, mp);
}

double RocketKinematics::computeYawAcceleration(double total_torque,
                                                const MassProperties& mp) const {
    double alpha = total_torque / mp.I_zz;
    const double MAX_ALPHA = 100.0;  // rad/s^2, same safety clamp as pitch
    return std::max(-MAX_ALPHA, std::min(MAX_ALPHA, alpha));
}

// Integrate: torque -> yaw acceleration -> yaw rate -> yaw angle. Rate is
// clamped the same way pitch's is (numerical safety net); the angle
// itself just wraps into [0, 2*pi) -- it's a compass azimuth, not a
// bounded tilt, so there's no ~85 degree ceiling to clamp to.
void RocketKinematics::updateYawDynamics(RocketState& next, const RocketState& state) const {
    MassProperties mp = mass_model_.computeAt(state.mass);
    mp.cp_location_cm = cp_location_cm_;

    double total_torque = computeTotalYawTorque(state, mp);
    double alpha = computeYawAcceleration(total_torque, mp);

    next.angular_vel(2) = state.angular_vel(2) + alpha * config_.dt;

    const double MAX_ANGULAR_VEL = 10.0;  // rad/s
    next.angular_vel(2) = std::max(-MAX_ANGULAR_VEL, std::min(MAX_ANGULAR_VEL, next.angular_vel(2)));

    next.orientation(2) = fmod(state.orientation(2) + next.angular_vel(2) * config_.dt, 2 * M_PI);
}

// Same mass properties/torque math as updateYawDynamics, but returns the
// breakdown instead of integrating it -- for logging/plotting, mirroring
// computePitchTorques().
YawTorques RocketKinematics::computeYawTorques(const RocketState& state) const {
    MassProperties mp = mass_model_.computeAt(state.mass);
    mp.cp_location_cm = cp_location_cm_;

    YawTorques torques{};
    torques.gravity = computeYawGravityTorque(state, mp);
    torques.aerodynamic = computeYawAeroMoment(state, mp);
    torques.damping = computeYawDampingTorque(state, mp);
    return torques;
}
