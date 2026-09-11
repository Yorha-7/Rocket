#pragma once

#include "gnc/sensors.hpp"
#include <Eigen/Dense>

// Point-the-nose-at-the-target guidance: no PID, no trajectory
// optimization, just aim thrust straight at a fixed point. TVC's own
// actuator lag is the only closed-loop dynamics here on purpose --
// stacking a second controller on top would compound lag on lag.
class Navigation {
public:
    // target_position: world-frame aim point, fixed for the flight
    // (hardcoded by the caller, e.g. main.cpp -- no retargeting yet).
    // Throws if the target is on/near the ground -- aiming the nose (and
    // thrust) there is unsurvivable, not just "out of range."
    explicit Navigation(const Eigen::Vector3d& target_position);

    // Body-frame direction to feed ThrustVectorControl::commandForceDirection
    // so thrust points at the target. Stateless: no memory between calls.
    Eigen::Vector3d computeTvcTarget(const sensors::Gps& gps, const sensors::Gyro& gyro) const;

private:
    static constexpr double MIN_TARGET_ALTITUDE_M = 10.0;  // ground-safety floor, meters
    Eigen::Vector3d target_position_;                      // world frame, meters
};
