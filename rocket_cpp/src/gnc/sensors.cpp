#include "gnc/sensors.hpp"
#include "sim/rocket_kinematics.hpp"
#include <cmath>

namespace sensors {

void Gyro::update(const RocketState& state, const RocketState& prev_state, double dt) {
    const double g0 = 9.80665;

    // Total (kinematic) acceleration, world frame, from the change in
    // velocity -- then subtract gravity to get what an accelerometer's
    // proof mass actually feels (a free-falling vehicle reads ~0, not
    // -g, on every axis -- gravity acts on the proof mass exactly the
    // same as the case around it, so there's no relative force to sense).
    Eigen::Vector3d total_accel_world = Eigen::Vector3d::Zero();
    if (dt > 1e-9) {
        total_accel_world = (state.velocity - prev_state.velocity) / dt;
    }
    Eigen::Vector3d gravity_world(0, 0, -g0);
    Eigen::Vector3d proper_accel_world = total_accel_world - gravity_world;

    // Rotate world -> body using the same DCM the integrator itself uses
    // (transpose of body->world, since it's a pure rotation).
    Eigen::Matrix3d R = RocketKinematics::rocketToNedFrame(state);
    accel_body_ = R.transpose() * proper_accel_world;

    angular_accel_ = Eigen::Vector3d::Zero();
    if (dt > 1e-9) {
        angular_accel_ = (state.angular_vel - prev_state.angular_vel) / dt;
    }

    orientation_ = state.orientation;
}

void Baro::update(const RocketState& state) {
    // Same troposphere ISA formula as AerodynamicsModel::getDensity() --
    // duplicated rather than shared because Sensors is deliberately
    // decoupled from the aerodynamics/kinematics internals (a real flight
    // computer's baro driver doesn't reach into the vehicle's own drag
    // model either). Only handles the troposphere (<=11km) -- fine for
    // this vehicle's flight envelope, same scope as everywhere else in
    // this iteration.
    const double T0 = 288.15, P0 = 101325.0, LAPSE_RATE = -0.0065;
    const double G0 = 9.80665, R_GAS = 287.058;

    double altitude = std::max(0.0, state.position(2));
    double T = T0 + LAPSE_RATE * altitude;
    double exponent = -G0 / (LAPSE_RATE * R_GAS);
    pressure_pa_ = P0 * std::pow(T / T0, exponent);

    // A real baro would now invert pressure back to altitude -- but with
    // no sensor noise/bias modeled yet, that inversion returns exactly
    // this altitude back out, so there's nothing to gain by actually
    // doing the (numerically fussier) inverse formula yet.
    altitude_m_ = altitude;
}

void Gps::update(const RocketState& state) {
    position_ = state.position;
}

}  // namespace sensors
