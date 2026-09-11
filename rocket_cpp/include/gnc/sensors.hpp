#pragma once

#include "sim/rocket_types.hpp"
#include <Eigen/Dense>

// Simulated flight-computer sensors: each one reads its value straight
// from the vehicle's true kinematics (RocketState) -- no noise, bias, or
// drift modeled yet, just the same "update() then read...()" shape real
// sensor drivers use, so those can be layered in later without changing
// how callers use these classes. One namespace, one class per physical
// sensor (not one do-everything class) -- mirrors how real flight
// computers wire up a handful of independent parts, not a monolith.
namespace sensors {

// The IMU: accelerometer + gyroscope (a magnetometer is common on real
// IMU chips too, but left out for now -- nothing needs heading yet).
class Gyro {
public:
    // Computes this step's reading from the true kinematics. state/
    // prev_state + dt give the accelerometer and gyroscope their readings
    // (both are rate-of-change sensors -- they need two samples, not one).
    void update(const RocketState& state, const RocketState& prev_state, double dt);

    // Proper (specific) acceleration, body frame, m/s^2 -- what an
    // accelerometer actually measures. NOT the same as total kinematic
    // acceleration: an accelerometer's proof mass doesn't feel gravity
    // (a free-falling vehicle reads ~0 on every axis), so this is total
    // acceleration with gravity subtracted back out before rotating into
    // body frame.
    Eigen::Vector3d readAccel() const { return accel_body_; }

    // Angular acceleration, body-ish frame (finite difference of
    // angular_vel between the two states), rad/s^2.
    Eigen::Vector3d readAngularAccel() const { return angular_accel_; }

    // Angular rate (p, q, r), body frame, rad/s -- what a gyroscope
    // actually measures directly (unlike accel/angular-accel above, no
    // finite difference needed: rate is already a first-derivative
    // quantity RocketState carries natively).
    Eigen::Vector3d readAngularVel() const { return angular_vel_; }

    // True orientation (roll, pitch, yaw), rad -- a real IMU doesn't hand
    // you absolute attitude for free, that takes integrating/fusing the
    // rates above (or a magnetometer/star tracker). That estimator isn't
    // built yet, so this is a direct pass-through of the true value, same
    // simplification every other "read" in this file already makes.
    Eigen::Vector3d readOrientation() const { return orientation_; }

private:
    Eigen::Vector3d accel_body_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_accel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d orientation_ = Eigen::Vector3d::Zero();
};

// Barometric altimeter: measures pressure, reports the altitude a real
// baro would derive from it via the standard ISA formula. Our own
// pressure model and our own "true" altitude are already self-consistent,
// so -- with no sensor error modeled yet -- the derived altitude comes
// back exactly equal to the true one; the point is the shape (pressure
// in, altitude out), not an error source yet.
class Baro {
public:
    void update(const RocketState& state);
    double readPressure() const { return pressure_pa_; }
    double readAltitude() const { return altitude_m_; }

private:
    double pressure_pa_ = 0.0;
    double altitude_m_ = 0.0;
};

// GPS: world-frame position fix. Real GPS also reports velocity, but
// nothing needs that yet.
class Gps {
public:
    void update(const RocketState& state);
    Eigen::Vector3d readPosition() const { return position_; }

private:
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
};

}  // namespace sensors
