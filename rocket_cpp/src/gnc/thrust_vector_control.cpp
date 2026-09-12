#include "gnc/thrust_vector_control.hpp"
#include <cmath>
#include <algorithm>

ThrustVectorControl::ThrustVectorControl() = default;

// ##### commandForceDirection() #####
// Goal: turn "I want the FORCE to end up pointing this way" into a
// target angle for each of the two gimbal servos. Nozzle direction is
// built as nozzle_dir = (sin(gx), cos(gx)*sin(gy), -cos(gx)*cos(gy)) --
// gx tips the nozzle toward +X (rotating the neutral -Z direction about
// body Y), gy then tips THAT toward +Y (rotating about body X) --
// exactly how a two-axis gimbal ring with two independent servos really
// works. Force is opposite the nozzle (Newton's third law), so this
// solves the inverse of that formula and clamps each axis to its
// physical travel limit.
void ThrustVectorControl::commandForceDirection(const Eigen::Vector3d& target_force_dir) {
    double norm = target_force_dir.norm();
    if (norm < 1e-9) {
        target_gimbal_pitch_rad_ = 0.0;
        target_gimbal_yaw_rad_ = 0.0;
        return;
    }

    Eigen::Vector3d nozzle_dir = -target_force_dir / norm;

    double gx = std::asin(std::max(-1.0, std::min(1.0, nozzle_dir.x())));
    double cos_gx = std::cos(gx);
    double gy = (std::abs(cos_gx) > 1e-6)
        ? std::asin(std::max(-1.0, std::min(1.0, nozzle_dir.y() / cos_gx)))
        : 0.0;

    double max_rad = MAX_GIMBAL_DEG * M_PI / 180.0;
    target_gimbal_pitch_rad_ = std::max(-max_rad, std::min(max_rad, gx));
    target_gimbal_yaw_rad_ = std::max(-max_rad, std::min(max_rad, gy));
}

// ##### step() #####
// Goal: ease both actuators toward their target, first-order-lag style
// -- each step closes a fixed FRACTION (dt/tau) of whatever error is
// left, the same shape a real servo's response takes instead of
// teleporting to the setpoint. Keep dt well under tau (as the sim's own
// integrator dt already is) or this simple Euler form can overshoot.
void ThrustVectorControl::step(double dt) {
    gimbal_pitch_rad_ += (target_gimbal_pitch_rad_ - gimbal_pitch_rad_) * (dt / TAU_PITCH_S);
    gimbal_yaw_rad_ += (target_gimbal_yaw_rad_ - gimbal_yaw_rad_) * (dt / TAU_YAW_S);
}

Eigen::Vector3d ThrustVectorControl::currentNozzleDirection() const {
    return nozzleDirectionFromAngles(gimbal_pitch_rad_, gimbal_yaw_rad_);
}

// Goal: the forward geometry commandForceDirection() solves the inverse
// of -- turn a pair of gimbal angles into the actual unit-vector nozzle
// direction. Always a unit vector; (0,0,-1) (straight back) when both
// angles are 0.
Eigen::Vector3d ThrustVectorControl::nozzleDirectionFromAngles(double gimbal_pitch_rad, double gimbal_yaw_rad) {
    double sx = std::sin(gimbal_pitch_rad), cx = std::cos(gimbal_pitch_rad);
    double sy = std::sin(gimbal_yaw_rad), cy = std::cos(gimbal_yaw_rad);
    return Eigen::Vector3d(sx, cx * sy, -cx * cy);
}

Eigen::Vector3d ThrustVectorControl::currentForce(double thrust) const {
    return -thrust * currentNozzleDirection();
}

double ThrustVectorControl::currentGimbalPitchDeg() const {
    return gimbal_pitch_rad_ * 180.0 / M_PI;
}

double ThrustVectorControl::currentGimbalYawDeg() const {
    return gimbal_yaw_rad_ * 180.0 / M_PI;
}

double ThrustVectorControl::targetGimbalPitchDeg() const {
    return target_gimbal_pitch_rad_ * 180.0 / M_PI;
}

double ThrustVectorControl::targetGimbalYawDeg() const {
    return target_gimbal_yaw_rad_ * 180.0 / M_PI;
}
