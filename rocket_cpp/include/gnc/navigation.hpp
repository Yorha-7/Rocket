#pragma once

#include "gnc/sensors.hpp"
#include <Eigen/Dense>
#include <cmath>

// Point-the-nose-at-the-target guidance, shaped by a PID controller per
// axis (pitch-plane, yaw-plane) instead of commanding the raw angular
// error straight through. TVC's own actuator lag models the physical
// SERVO -- it has no feedback of its own (see thrust_vector_control.hpp),
// so this is where the actual closed-loop control law lives: P reacts to
// how far off-target the nose is, I cleans up whatever P alone can't
// null out, D would damp against the vehicle's own rotation rate
// (straight from the gyro, not a differentiated angle -- differentiating
// a noisy error amplifies noise, feeding back a rate sensor's own direct
// reading doesn't) -- but empirically (see KD's comment) it's left at
// zero here, since the actuator's own first-order lag already fills that
// role.
class Navigation {
public:
    // target_position: world-frame aim point, fixed for the flight
    // (hardcoded by the caller, e.g. main.cpp -- no retargeting yet).
    // Throws if the target is on/near the ground -- aiming the nose (and
    // thrust) there is unsurvivable, not just "out of range."
    explicit Navigation(const Eigen::Vector3d& target_position);

    // Body-frame direction to feed ThrustVectorControl::commandForceDirection
    // so thrust points at the target, PID-shaped as described above. dt is
    // this step's integration interval -- needed for the integral term
    // (and its anti-windup clamp). Not const: the integral accumulators
    // are real state that persists step to step.
    Eigen::Vector3d computeTvcTarget(const sensors::Gps& gps, const sensors::Gyro& gyro, double dt);

private:
    static constexpr double MIN_TARGET_ALTITUDE_M = 10.0;  // ground-safety floor, meters

    // ---- Tuned here directly -- no config file for now ----
    // Kp=1, Ki=0, Kd=0 reproduces the old direct-passthrough behavior
    // exactly (the command WAS the raw angular error, unity gain). Ki/Kd
    // are on top of that baseline, not a replacement for it.
    //
    // Gains below came from a grid search against the current test
    // scenario's closest approach to NAV_TARGET (main.cpp), not analytic
    // tuning -- two findings from that search:
    //   - KD hurt monotonically at every KI tested (closest-approach
    //     distance only got worse as KD grew). Makes sense: the actuator
    //     lag (ThrustVectorControl's own first-order response) already
    //     acts as this loop's damping element, so feeding back gyro rate
    //     on top of it fights the correction instead of smoothing it --
    //     left at 0 rather than force a term that measurably hurts.
    //   - KI alone helped a lot, but keeps helping well past where the
    //     gimbal command stops looking "sane": by KI=0.6 the pitch
    //     actuator is oscillating (~2-3 Hz, tens of degrees swing) late
    //     in flight. That's harmless in THIS scenario because it happens
    //     after the motor's thrust has already tapered off (no thrust =
    //     gimbal angle has nothing to redirect), but it's real
    //     integral-windup character, not a clean controlled response --
    //     a longer-burning motor would turn this into an actual problem.
    static constexpr double KP = 1.0;
    static constexpr double KI = 0.6;
    static constexpr double KD = 0.0;

    // Anti-windup: caps how much the integral term alone can contribute,
    // so a long saturated stretch (actuator pinned at its travel limit,
    // error not shrinking) can't leave a huge accumulated integral that
    // then overshoots once the error finally starts closing. Bound is
    // one full actuator swing's worth of command -- mirrors
    // ThrustVectorControl::MAX_GIMBAL_DEG (30 deg) as a separate
    // constant rather than a shared one, same "deliberately decoupled"
    // pattern sensors.cpp already uses for its own ISA formula copy.
    static constexpr double MAX_GIMBAL_RAD = 30.0 * M_PI / 180.0;

    Eigen::Vector3d target_position_;  // world frame, meters

    // Integral accumulators, one per axis -- real memory between calls,
    // unlike the rest of this class.
    double integral_pitch_ = 0.0;
    double integral_yaw_ = 0.0;
};
