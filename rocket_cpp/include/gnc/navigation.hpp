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
//
// The PID above doesn't read raw GPS -- it reads a FUSED position
// estimate (see estimatePosition()) that blends the noisy GPS fix with
// the accelerometer's own double-integrated motion, an alpha-beta (g-h)
// filter. Why that's there: GPS's own noise (gnc/sensors.hpp) has a
// ~60s correlation time, comparable to this vehicle's whole ~25s flight,
// so within a single flight it looks less like jitter and more like a
// near-constant few-meter miscalibration -- no amount of PID retuning
// fixes that (see the empirical finding in Staging Notes), and the fused
// estimate is a genuine step up from feeding raw GPS straight into a
// control law, though it CANNOT invent an independent ground truth GPS
// doesn't have either -- it mainly buys smoother short-term tracking
// between GPS corrections, not immunity to a biased fix.
//
// Stays neutral below ACTIVATION_ALTITUDE_M: right off the pad, the
// rocket is slow and barely off vertical, so a tiny geometric angle
// error swings the PID command toward full gimbal deflection -- exactly
// the wrong moment for that (rail/tower still nearby, no airspeed yet
// for TVC's authority to mean much anyway). Guidance only takes over
// once the vehicle has cleared that floor.
class Navigation {
public:
    // target_position: world-frame aim point, fixed for the flight
    // (hardcoded by the caller, e.g. main.cpp -- no retargeting yet).
    // dt: the sim's own integration step -- needed once, up front, to
    // size the position estimator's filter gains (see estimatePosition).
    // Throws if the target is on/near the ground -- aiming the nose (and
    // thrust) there is unsurvivable, not just "out of range."
    Navigation(const Eigen::Vector3d& target_position, double dt);

    // Body-frame direction to feed ThrustVectorControl::commandForceDirection
    // so thrust points at the target, PID-shaped as described above. dt is
    // this step's integration interval -- needed for the integral term
    // (and its anti-windup clamp) and the position estimator. Not const:
    // the integral accumulators and the position filter are real state
    // that persists step to step.
    Eigen::Vector3d computeTvcTarget(const sensors::Gps& gps, const sensors::Gyro& gyro, double dt);

private:
    static constexpr double MIN_TARGET_ALTITUDE_M = 10.0;  // ground-safety floor, meters

    // Altitude the VEHICLE must clear before guidance activates (not to
    // be confused with MIN_TARGET_ALTITUDE_M above, which is about where
    // the TARGET sits, checked once at construction). Checked every
    // step, against the fused position estimate -- see class doc comment.
    static constexpr double ACTIVATION_ALTITUDE_M = 2.0;

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
    //   - Re-verified after sensor noise (and the position filter below)
    //     were added: KI=0.6 is still the best performer -- accuracy
    //     came back statistically unchanged (~48m closest approach either
    //     way, averaged over repeated noise draws), so this wasn't a
    //     gain-tuning problem to begin with. See Staging Notes.
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

    // ---- Position estimator: alpha-beta (g-h) filter ----
    // Two-state (position, velocity) tracker, predicting each step from
    // the accelerometer's own measured (noisy) acceleration rather than
    // assuming constant velocity the way a textbook alpha-beta filter
    // does -- "acceleration-aided" prediction, since a real measurement
    // is available instead of nothing. Corrected each step by GPS's
    // position residual, with gains split between position (alpha) and
    // velocity (beta) exactly like the classic filter (see .cpp).
    //
    // Gains are derived, not guessed: the steady-state alpha/beta
    // relation for a g-h filter (Wikipedia, "Alpha beta filter" --
    // lambda = sigma_process*dt^2/sigma_meas, then a closed form for
    // alpha/beta from that) computed once at construction from this
    // project's own real numbers -- GPS's steady-state sigma
    // (gnc/sensors.hpp) as the measurement noise, and the accelerometer's
    // white-noise std as the process-uncertainty term (a reasonable
    // adaptation for THIS filter's acceleration-aided prediction, not a
    // literal textbook constant-velocity case). Horizontal (x,y) and
    // vertical (z) axes get separate gains since GPS's own noise differs
    // between them (see HORIZONTAL_SIGMA_M/VERTICAL_SIGMA_M).
    // These mirror sensors::Gps/Gyro's own (private) noise constants --
    // duplicated rather than exposed, same "deliberately decoupled"
    // pattern MAX_GIMBAL_RAD above already uses for ThrustVectorControl's
    // constant.
    static constexpr double GPS_HORIZONTAL_SIGMA_M = 2.5;
    static constexpr double GPS_VERTICAL_SIGMA_M = 2.5 * 1.7;
    static constexpr double ACCEL_NOISE_STD_MPS2 = 0.008 * 9.80665;

    static void computeAlphaBeta(double sigma_process, double sigma_meas, double dt,
                                  double& alpha, double& beta);
    Eigen::Vector3d estimatePosition(const sensors::Gps& gps, const sensors::Gyro& gyro,
                                      const Eigen::Matrix3d& R, double dt);

    double horizontal_alpha_;
    double horizontal_beta_;
    double vertical_alpha_;
    double vertical_beta_;

    Eigen::Vector3d fused_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d fused_velocity_ = Eigen::Vector3d::Zero();
    bool position_filter_initialized_ = false;

    Eigen::Vector3d target_position_;  // world frame, meters

    // Integral accumulators, one per axis -- real memory between calls,
    // unlike the rest of this class.
    double integral_pitch_ = 0.0;
    double integral_yaw_ = 0.0;
};
