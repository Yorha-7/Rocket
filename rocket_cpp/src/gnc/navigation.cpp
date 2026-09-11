#include "gnc/navigation.hpp"
#include "sim/rocket_kinematics.hpp"
#include <stdexcept>
#include <algorithm>
#include <cmath>

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

Eigen::Vector3d Navigation::computeTvcTarget(const sensors::Gps& gps, const sensors::Gyro& gyro, double dt) {
    // Below the activation floor: hold neutral and keep the integrators
    // at zero, so whenever guidance DOES activate it starts from a clean
    // slate instead of carrying windup accumulated while it was sitting
    // idle on the pad (see class doc comment for why this floor exists).
    if (gps.readPosition().z() < ACTIVATION_ALTITUDE_M) {
        integral_pitch_ = 0.0;
        integral_yaw_ = 0.0;
        return Eigen::Vector3d(0, 0, 1);
    }

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
    // rotation) takes our world-frame aim vector into body frame.
    Eigen::Vector3d dir_body = (R.transpose() * to_target_world).normalized();

    // Decompose into per-axis angular error, same asin geometry
    // ThrustVectorControl uses internally (nozzle deflection vs. force
    // direction) -- applied here to the TARGET direction itself, i.e.
    // how far off dead-ahead (body +Z, the nose) the target currently
    // sits, split into the pitch plane (X-Z) and yaw plane (Y-Z).
    double err_pitch = std::asin(std::max(-1.0, std::min(1.0, dir_body.x())));
    double cos_p = std::cos(err_pitch);
    double err_yaw = (std::abs(cos_p) > 1e-6)
        ? std::asin(std::max(-1.0, std::min(1.0, dir_body.y() / cos_p)))
        : 0.0;

    // Integral: accumulate error over time, clamped so a long saturated
    // stretch can't build up more windup than one actuator swing is
    // worth (see MAX_GIMBAL_RAD's declaration comment).
    integral_pitch_ += err_pitch * dt;
    integral_yaw_ += err_yaw * dt;
    double max_integral = (KI > 1e-9) ? (MAX_GIMBAL_RAD / KI) : 0.0;
    if (KI > 1e-9) {
        integral_pitch_ = std::max(-max_integral, std::min(max_integral, integral_pitch_));
        integral_yaw_ = std::max(-max_integral, std::min(max_integral, integral_yaw_));
    }

    // Derivative: straight from the gyro's own rate reading, not a
    // differentiated (noisy) error -- see class doc comment. Sign is
    // negative because increasing pitch/yaw rate in the direction that
    // CLOSES the error should reduce the command (anticipatory braking,
    // not fighting the correction that's already happening).
    Eigen::Vector3d rate = gyro.readAngularVel();
    double d_pitch = -rate.y();
    double d_yaw = -rate.z();

    double u_pitch = KP * err_pitch + KI * integral_pitch_ + KD * d_pitch;
    double u_yaw = KP * err_yaw + KI * integral_yaw_ + KD * d_yaw;

    // Reconstruct a direction vector from the PID-shaped angles, using
    // the exact inverse of the decomposition above -- with KP=1, KI=KD=0
    // this round-trips to precisely dir_body, i.e. the old direct-
    // passthrough behavior. ThrustVectorControl::commandForceDirection
    // re-normalizes and re-decomposes this on the other end (and clamps
    // to the actuator's own physical travel limit) -- this class doesn't
    // need to duplicate that clamp, just hand over the shaped direction.
    double su = std::sin(u_pitch), cu = std::cos(u_pitch);
    double sy = std::sin(u_yaw), cy = std::cos(u_yaw);
    return Eigen::Vector3d(su, cu * sy, cu * cy);
}
