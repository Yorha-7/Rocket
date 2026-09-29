#include "sim/rocket_kinematics.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

// ##### Yaw dynamics, the big picture #####
// Goal: exact mirror of pitch_dynamics.cpp, one plane over -- body Y-Z
// instead of body X-Z, sideslip (beta) instead of angle of attack
// (alpha), same shared Cn_alpha/CP/CG/I_zz (I_zz = I_yy, axisymmetric
// body). Before this file existed, yaw just held whatever init_yaw set
// it to for the whole flight -- this gives it the same real
// weathercocking behavior pitch has always had, so Y-axis steering
// (TVC/Navigation) gets vehicle-attitude assistance too, not just X.
//
// Both pitch and yaw wrap (fmod) rather than clamp their angle -- neither
// is physically bounded to a narrow range (a vehicle can legitimately fly
// nose-down, inverted, or through any azimuth). Pitch used to hard-clamp
// near +-90 degrees; see pitch_dynamics.cpp's updatePitchDynamics() for
// why that turned out to be a real problem, not a safe default.

// ##### computeYawGravityTorque() #####
// Goal: always zero, same reasoning as pitch's version -- no net torque
// about the CG from a uniform field, regardless of which axis.
double RocketKinematics::computeYawGravityTorque(const RocketState& state,
                                                 const MassProperties& mp) const {
    return 0.0;
}

// ##### computeYawAeroMoment() #####
// Goal: the yaw-plane restoring torque -- air pushing on the CP trying
// to swing the nose back toward the direction of travel, same physics as
// pitch's aero moment, just using sideslip (beta) instead of angle of
// attack (alpha).
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

// ##### computeYawDampingTorque() #####
// Goal: resist whatever yaw rotation is happening, proportional to how
// fast it's spinning -- same weathervane damping as pitch, mirrored.
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

// Goal: same Newton's-second-law-for-rotation clamp as pitch, mirrored
// to the yaw axis.
double RocketKinematics::computeYawAcceleration(double total_torque,
                                                const MassProperties& mp) const {
    double alpha = total_torque / mp.I_zz;
    const double MAX_ALPHA = 100.0;  // rad/s^2, same safety clamp as pitch
    return std::max(-MAX_ALPHA, std::min(MAX_ALPHA, alpha));
}

// ##### updateYawDynamics() #####
// Goal: integrate torque -> yaw acceleration -> yaw rate -> yaw angle.
// Rate is clamped the same numerical-safety way pitch's is; the angle
// itself just wraps into [0, 2*pi) since it's a compass azimuth, not a
// bounded tilt -- there's no "past vertical" concept for yaw the way
// there historically was (wrongly) for pitch.
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

// ##### computeYawTorques() #####
// Goal: same math as updateYawDynamics, but return the breakdown instead
// of integrating it -- for logging/plotting, mirroring computePitchTorques().
YawTorques RocketKinematics::computeYawTorques(const RocketState& state) const {
    MassProperties mp = mass_model_.computeAt(state.mass);
    mp.cp_location_cm = cp_location_cm_;

    YawTorques torques{};
    torques.gravity = computeYawGravityTorque(state, mp);
    torques.aerodynamic = computeYawAeroMoment(state, mp);
    torques.damping = computeYawDampingTorque(state, mp);
    return torques;
}
