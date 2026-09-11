#include "gnc/navigation.hpp"
#include "sim/rocket_kinematics.hpp"
#include <stdexcept>

Navigation::Navigation(const Eigen::Vector3d& target_position) : target_position_(target_position) {
    if (target_position.z() <= MIN_TARGET_ALTITUDE_M) {
        // Fail at construction, not mid-flight -- a bad target should
        // never even get to fly.
        throw std::invalid_argument(
            "Navigation target is on/near the ground (z=" + std::to_string(target_position.z()) +
            "m, minimum " + std::to_string(MIN_TARGET_ALTITUDE_M) +
            "m) -- aiming the nose there means aiming thrust into the ground.");
    }
}

Eigen::Vector3d Navigation::computeTvcTarget(const sensors::Gps& gps, const sensors::Gyro& gyro) const {
    // Straight-line vector from where GPS says we are to the target,
    // still in world frame (north/east/up) at this point.
    Eigen::Vector3d to_target_world = target_position_ - gps.readPosition();

    if (to_target_world.norm() < 1e-6) {
        return Eigen::Vector3d(0, 0, 1);  // already there -- hold neutral, nothing to aim at
    }

    // Only orientation matters below (rocketToNedFrame is a pure function
    // of it) -- build a throwaway state rather than a one-off overload.
    RocketState orientation_state{};
    orientation_state.position = Eigen::Vector3d::Zero();      // unused by rocketToNedFrame
    orientation_state.velocity = Eigen::Vector3d::Zero();      // unused
    orientation_state.angular_vel = Eigen::Vector3d::Zero();   // unused
    orientation_state.mass = 0.0;                              // unused
    orientation_state.orientation = gyro.readOrientation();    // the one field that matters

    // Body<-world rotation for the CURRENT attitude (shared formula, not
    // a second hand-copied DCM -- see rocket_kinematics.hpp).
    Eigen::Matrix3d R = RocketKinematics::rocketToNedFrame(orientation_state);

    // R rotates body->world, so its transpose (= inverse, R is a pure
    // rotation) takes our world-frame aim vector into body frame, which
    // is what commandForceDirection() expects.
    return R.transpose() * to_target_world;
}
