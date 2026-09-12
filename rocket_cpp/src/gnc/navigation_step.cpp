#include "gnc/navigation.hpp"
#include "sim/rocket_kinematics.hpp"
#include <algorithm>
#include <cmath>

namespace navigation {

// ##### Constructor #####
// Goal: lock in the initial aim point and gains, and size the
// position-filter gains once from the sim's own dt and this project's
// real sensor noise numbers. No ground-safety check here -- that moved
// to Navigation's constructor (see navigation.hpp class comment); this
// class trusts whatever target it's given.
NavigationStep::NavigationStep(const Eigen::Vector3d& target_position, double dt,
                               double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), target_position_(target_position) {
    computeAlphaBeta(ACCEL_NOISE_STD_MPS2, GPS_HORIZONTAL_SIGMA_M, dt, horizontal_alpha_, horizontal_beta_);
    computeAlphaBeta(ACCEL_NOISE_STD_MPS2, GPS_VERTICAL_SIGMA_M, dt, vertical_alpha_, vertical_beta_);
}

// ##### computeAlphaBeta() #####
// Goal: derive the filter's own gain pair instead of hand-picking one --
// the steady-state alpha/beta relation for a g-h filter, given the ratio
// of process uncertainty to measurement uncertainty (lambda). Balances
// how much to trust the accelerometer's prediction against how much to
// trust each GPS correction.
void NavigationStep::computeAlphaBeta(double sigma_process, double sigma_meas, double dt,
                                       double& alpha, double& beta) {
    double lambda = sigma_process * dt * dt / sigma_meas;
    double r = (4.0 + lambda - std::sqrt(8.0 * lambda + lambda * lambda)) / 4.0;
    alpha = 1.0 - r * r;
    beta = 2.0 * (2.0 - alpha) - 4.0 * std::sqrt(1.0 - alpha);
}

// ##### estimatePosition() #####
// Goal: blend the noisy GPS fix with the accelerometer's own double-
// integrated motion -- predict from the accelerometer each step
// ("acceleration-aided", since a real measurement exists instead of
// assuming constant velocity), then correct both position AND velocity
// from how far off that prediction was from GPS. R is the current
// body->world rotation, passed in from the caller so it isn't rebuilt twice.
Eigen::Vector3d NavigationStep::estimatePosition(const sensors::Gps& gps, const sensors::Gyro& gyro,
                                                  const Eigen::Matrix3d& R, double dt) {
    if (!position_filter_initialized_) {
        // Goal: bootstrap from the first GPS fix -- this class has no
        // other source of an absolute starting position, it only ever
        // sees sensor readings, never the launch pad's true coordinates.
        fused_position_ = gps.readPosition();
        fused_velocity_ = Eigen::Vector3d::Zero();
        position_filter_initialized_ = true;
        return fused_position_;
    }

    // Goal: undo what Gyro::update() did to build readAccel() -- rotate
    // the (noisy) proper acceleration back to world frame, add gravity
    // back, and recover total kinematic acceleration, the quantity that
    // actually integrates into velocity/position.
    const double g0 = 9.80665;
    Eigen::Vector3d gravity_world(0, 0, -g0);
    Eigen::Vector3d total_accel_world = R * gyro.readAccel() + gravity_world;

    // Goal: predict from the accelerometer alone -- accurate over one
    // short dt, but drifts without bound if never corrected (bias,
    // double-integrated, grows unchecked).
    fused_velocity_ += total_accel_world * dt;
    Eigen::Vector3d predicted_position = fused_position_ + fused_velocity_ * dt;

    // Goal: correct BOTH position and velocity from the GPS residual --
    // the classic alpha-beta update. Correcting velocity too (not just
    // position) is what keeps fused_velocity_ from drifting away
    // unbounded between GPS corrections.
    Eigen::Vector3d residual = gps.readPosition() - predicted_position;
    Eigen::Vector3d alpha(horizontal_alpha_, horizontal_alpha_, vertical_alpha_);
    Eigen::Vector3d beta(horizontal_beta_, horizontal_beta_, vertical_beta_);
    fused_position_ = predicted_position + alpha.cwiseProduct(residual);
    fused_velocity_ = fused_velocity_ + beta.cwiseProduct(residual) / dt;

    return fused_position_;
}

// ##### computeTvcTarget() #####
// Goal: the guidance decision itself -- read the current position/
// attitude, work out the angle from the nose to the current target,
// shape it through a PID, and hand back a body-frame direction for TVC
// to aim at.
Eigen::Vector3d NavigationStep::computeTvcTarget(const sensors::Gps& gps, const sensors::Gyro& gyro, double dt) {
    // Goal: get the current attitude once -- needed both to fuse the
    // accelerometer into the position estimate below and to rotate the
    // target direction into body frame further down.
    RocketState orientation_state{};
    orientation_state.position = Eigen::Vector3d::Zero();      // unused by rocketToNedFrame
    orientation_state.velocity = Eigen::Vector3d::Zero();      // unused
    orientation_state.angular_vel = Eigen::Vector3d::Zero();   // unused
    orientation_state.mass = 0.0;                              // unused
    orientation_state.orientation = gyro.readOrientation();    // the one field that matters
    Eigen::Matrix3d R = RocketKinematics::rocketToNedFrame(orientation_state);

    // Goal: use the FUSED GPS+accelerometer estimate, not the raw GPS
    // fix -- see estimatePosition() and this class's own doc comment.
    Eigen::Vector3d position = estimatePosition(gps, gyro, R, dt);

    // Goal: stay neutral below the activation floor, and reset the
    // integrators while doing so -- so whenever guidance DOES activate
    // it starts clean instead of carrying windup from sitting idle.
    if (position.z() < ACTIVATION_ALTITUDE_M) {
        integral_pitch_ = 0.0;
        integral_yaw_ = 0.0;
        return Eigen::Vector3d(0, 0, 1);
    }

    // Goal: get the straight-line vector from here to the current
    // target, still in world frame.
    Eigen::Vector3d to_target_world = target_position_ - position;

    if (to_target_world.norm() < 1e-6) {
        return Eigen::Vector3d(0, 0, 1);  // already there -- hold neutral, nothing to aim at
    }

    // Goal: convert that world-frame aim vector into BODY frame -- R
    // rotates body->world, so its transpose (its inverse, since it's a
    // pure rotation) does the reverse conversion.
    Eigen::Vector3d dir_body = (R.transpose() * to_target_world).normalized();

    // Goal: split that body-frame direction into a pitch-plane angle and
    // a yaw-plane angle -- how far off dead-ahead (body +Z, the nose)
    // the target currently sits, same asin geometry
    // ThrustVectorControl uses internally.
    double err_pitch = std::asin(std::max(-1.0, std::min(1.0, dir_body.x())));
    double cos_p = std::cos(err_pitch);
    double err_yaw = (std::abs(cos_p) > 1e-6)
        ? std::asin(std::max(-1.0, std::min(1.0, dir_body.y() / cos_p)))
        : 0.0;

    // Goal: accumulate the integral term, clamped so a long saturated
    // stretch can't build more windup than one actuator swing is worth.
    integral_pitch_ += err_pitch * dt;
    integral_yaw_ += err_yaw * dt;
    double max_integral = (ki_ > 1e-9) ? (MAX_GIMBAL_RAD / ki_) : 0.0;
    if (ki_ > 1e-9) {
        integral_pitch_ = std::max(-max_integral, std::min(max_integral, integral_pitch_));
        integral_yaw_ = std::max(-max_integral, std::min(max_integral, integral_yaw_));
    }

    // Goal: the derivative term, straight from the gyro's own rate
    // reading rather than differentiating a noisy angle. Negative sign:
    // rotating in the direction that's already CLOSING the error should
    // reduce the command (anticipatory braking), not add to it.
    Eigen::Vector3d rate = gyro.readAngularVel();
    double d_pitch = -rate.y();
    double d_yaw = -rate.z();

    double u_pitch = kp_ * err_pitch + ki_ * integral_pitch_ + kd_ * d_pitch;
    double u_yaw = kp_ * err_yaw + ki_ * integral_yaw_ + kd_ * d_yaw;

    // Goal: rebuild a direction vector from the PID-shaped angles, the
    // exact inverse of the split above -- with KP=1, KI=KD=0 this
    // round-trips to precisely dir_body, i.e. the old direct-passthrough
    // behavior. ThrustVectorControl re-normalizes/re-decomposes this on
    // its end (and clamps to the actuator's own travel limit), so this
    // class doesn't need to duplicate that clamp.
    double su = std::sin(u_pitch), cu = std::cos(u_pitch);
    double sy = std::sin(u_yaw), cy = std::cos(u_yaw);
    return Eigen::Vector3d(su, cu * sy, cu * cy);
}

}  // namespace navigation
