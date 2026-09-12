#pragma once

#include <Eigen/Dense>

// ##### ThrustVectorControl #####
// Goal: simulate the physical TVC hardware -- not a perfect, instant aim
// command. Two independent servos tip the nozzle off its neutral
// (straight-back, body -Z) position: one in the body X-Z plane, one in
// the body Y-Z plane. Real servos don't snap to a new angle, they ease
// into it, so each axis is its own first-order lag (see docs/
// axis_and_tvc_reference.png and README "Coordinate frames" for the full
// picture -- these are gimbal deflection angles, not the vehicle's own
// orientation pitch/yaw).
//
// Usage: commandForceDirection() sets what the actuators are aiming for;
// step(dt) actually advances them toward it; currentForce()/
// currentNozzleDirection() read out where they really are right now.
class ThrustVectorControl {
public:
    ThrustVectorControl();

    // Goal: aim so the FORCE on the vehicle ends up along
    // target_force_dir (body frame, need not be normalized). Only
    // updates the TARGET the actuators are easing toward -- call step()
    // to actually move them. Points the nozzle the opposite way
    // internally (Newton's third law) and clamps to the physical travel
    // limit.
    void commandForceDirection(const Eigen::Vector3d& target_force_dir);

    // Goal: advance both actuators by dt seconds, each closing a
    // fraction of its own remaining error toward its target.
    void step(double dt);

    // Goal: report where the nozzle is ACTUALLY pointed right now (unit
    // vector, body frame) -- lags behind the commanded target until the
    // actuators catch up.
    Eigen::Vector3d currentNozzleDirection() const;

    // Goal: the force this produces on the vehicle at a given thrust
    // magnitude.
    Eigen::Vector3d currentForce(double thrust) const;

    // Goal: expose the raw actuator angles -- what RocketState stores
    // each step so computeNetForce() can rebuild the nozzle direction
    // without touching this (possibly-since-moved-on) live object.
    double currentGimbalPitchRad() const { return gimbal_pitch_rad_; }
    double currentGimbalYawRad() const { return gimbal_yaw_rad_; }

    // Same, in degrees, for logging/plotting.
    double currentGimbalPitchDeg() const;
    double currentGimbalYawDeg() const;
    double targetGimbalPitchDeg() const;
    double targetGimbalYawDeg() const;

    // Goal: the same nozzle-direction geometry currentNozzleDirection()
    // uses internally, exposed as a static function so
    // RocketKinematics::computeNetForce can rebuild it from a saved
    // RocketState's angles instead of depending on this object's live
    // (possibly different) position.
    static Eigen::Vector3d nozzleDirectionFromAngles(double gimbal_pitch_rad, double gimbal_yaw_rad);

private:
    // ---- Tuned here directly -- no config file for now ----
    static constexpr double MAX_GIMBAL_DEG = 30.0;  // physical travel limit, each axis
    // A first-order lag settles (~99%) after about 5*tau, so 0.05s here
    // lands the actuator on target inside the 250ms requirement (~99% by
    // 5*0.05=0.25s, ~95% already by 3*0.05=0.15s).
    static constexpr double TAU_PITCH_S = 0.05;     // pitch-actuator first-order time constant
    static constexpr double TAU_YAW_S = 0.05;       // yaw-actuator time constant

    double target_gimbal_pitch_rad_ = 0.0;  // where the pitch actuator is heading
    double target_gimbal_yaw_rad_ = 0.0;    // where the yaw actuator is heading
    double gimbal_pitch_rad_ = 0.0;         // where the pitch actuator actually is
    double gimbal_yaw_rad_ = 0.0;           // where the yaw actuator actually is
};
