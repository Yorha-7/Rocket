#include "gnc/navigation.hpp"
#include "sim/rocket_kinematics.hpp"
#include <stdexcept>
#include <algorithm>
#include <cmath>

Navigation::Navigation(const Eigen::Vector3d& target_position, double dt) : target_position_(target_position) {
    if (target_position.z() <= MIN_TARGET_ALTITUDE_M) {
        // Fail at construction, not mid-flight -- a bad target should
        // never even get to fly.
        throw std::invalid_argument(
            "Navigation target is on/near the ground (z=" + std::to_string(target_position.z()) +
            "m, minimum " + std::to_string(MIN_TARGET_ALTITUDE_M) +
            "m) -- aiming the nose there means aiming thrust into the ground.");
    }

    // Position-estimator gains, sized once from this project's own real
    // sensor numbers -- see estimatePosition()'s doc comment.
    computeAlphaBeta(ACCEL_NOISE_STD_MPS2, GPS_HORIZONTAL_SIGMA_M, dt, horizontal_alpha_, horizontal_beta_);
    computeAlphaBeta(ACCEL_NOISE_STD_MPS2, GPS_VERTICAL_SIGMA_M, dt, vertical_alpha_, vertical_beta_);
}

// Steady-state alpha/beta relation for a g-h filter (Wikipedia, "Alpha
// beta filter"): given the ratio of process uncertainty to measurement
// uncertainty (lambda), this is the closed-form gain pair that balances
// noise rejection against tracking lag, instead of hand-picked constants.
void Navigation::computeAlphaBeta(double sigma_process, double sigma_meas, double dt,
                                   double& alpha, double& beta) {
    double lambda = sigma_process * dt * dt / sigma_meas;
    double r = (4.0 + lambda - std::sqrt(8.0 * lambda + lambda * lambda)) / 4.0;
    alpha = 1.0 - r * r;
    beta = 2.0 * (2.0 - alpha) - 4.0 * std::sqrt(1.0 - alpha);
}

// Blends the noisy GPS fix with the accelerometer's own double-integrated
// motion -- an alpha-beta (g-h) filter, "acceleration-aided": the
// prediction step uses the accelerometer's actual measured acceleration
// each step rather than assuming constant velocity the way a textbook
// alpha-beta filter does, since a real measurement is available instead
// of nothing. R is the current body->world rotation (shared with the
// caller so this doesn't rebuild it a second time).
Eigen::Vector3d Navigation::estimatePosition(const sensors::Gps& gps, const sensors::Gyro& gyro,
                                              const Eigen::Matrix3d& R, double dt) {
    if (!position_filter_initialized_) {
        // Bootstrap from the first GPS fix -- Navigation has no other
        // source of an absolute starting position, it only ever sees
        // sensor readings, never the launch pad's true coordinates.
        fused_position_ = gps.readPosition();
        fused_velocity_ = Eigen::Vector3d::Zero();
        position_filter_initialized_ = true;
        return fused_position_;
    }

    // Undo what Gyro::update() did to build readAccel(): rotate the
    // (noisy) proper acceleration back to world frame, then add gravity
    // back to recover total kinematic acceleration -- the quantity that
    // actually integrates into velocity/position.
    const double g0 = 9.80665;
    Eigen::Vector3d gravity_world(0, 0, -g0);
    Eigen::Vector3d total_accel_world = R * gyro.readAccel() + gravity_world;

    // Predict from the accelerometer alone -- accurate over one short
    // dt, but the double integration drifts without bound if left
    // uncorrected (accelerometer bias, double-integrated, is exactly the
    // kind of error that grows unchecked -- see Staging Notes).
    fused_velocity_ += total_accel_world * dt;
    Eigen::Vector3d predicted_position = fused_position_ + fused_velocity_ * dt;

    // Correct BOTH position and velocity from the GPS residual -- the
    // classic alpha-beta update. Correcting velocity too (not just
    // position) is what keeps fused_velocity_ from drifting away
    // unbounded between GPS corrections; a position-only blend would
    // still leave the velocity state free-running.
    Eigen::Vector3d residual = gps.readPosition() - predicted_position;
    Eigen::Vector3d alpha(horizontal_alpha_, horizontal_alpha_, vertical_alpha_);
    Eigen::Vector3d beta(horizontal_beta_, horizontal_beta_, vertical_beta_);
    fused_position_ = predicted_position + alpha.cwiseProduct(residual);
    fused_velocity_ = fused_velocity_ + beta.cwiseProduct(residual) / dt;

    return fused_position_;
}

Eigen::Vector3d Navigation::computeTvcTarget(const sensors::Gps& gps, const sensors::Gyro& gyro, double dt) {
    // Attitude is needed both to fuse the accelerometer into the position
    // estimate below and to rotate the target direction into body frame
    // further down -- computed once here, shared by both instead of
    // rebuilding it twice.
    RocketState orientation_state{};
    orientation_state.position = Eigen::Vector3d::Zero();      // unused by rocketToNedFrame
    orientation_state.velocity = Eigen::Vector3d::Zero();      // unused
    orientation_state.angular_vel = Eigen::Vector3d::Zero();   // unused
    orientation_state.mass = 0.0;                              // unused
    orientation_state.orientation = gyro.readOrientation();    // the one field that matters
    Eigen::Matrix3d R = RocketKinematics::rocketToNedFrame(orientation_state);

    // Fused GPS+accelerometer estimate, not the raw GPS fix -- see
    // estimatePosition()'s doc comment and the class doc comment for why.
    Eigen::Vector3d position = estimatePosition(gps, gyro, R, dt);

    // Below the activation floor: hold neutral and keep the integrators
    // at zero, so whenever guidance DOES activate it starts from a clean
    // slate instead of carrying windup accumulated while it was sitting
    // idle on the pad (see class doc comment for why this floor exists).
    if (position.z() < ACTIVATION_ALTITUDE_M) {
        integral_pitch_ = 0.0;
        integral_yaw_ = 0.0;
        return Eigen::Vector3d(0, 0, 1);
    }

    // Straight-line vector from the fused position estimate to the
    // target, still in world frame (north/east/up) at this point.
    Eigen::Vector3d to_target_world = target_position_ - position;

    if (to_target_world.norm() < 1e-6) {
        return Eigen::Vector3d(0, 0, 1);  // already there -- hold neutral, nothing to aim at
    }

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
