#pragma once

#include "gnc/sensors.hpp"
#include <Eigen/Dense>
#include <cmath>

// ##### Navigation, the big picture #####
// Goal: point the nose at a fixed target, shaped by a PID controller per
// axis (pitch-plane, yaw-plane) instead of commanding the raw angular
// error straight through. TVC's own actuator lag (see
// thrust_vector_control.hpp) models the physical servo -- it has no
// feedback loop of its own -- so this class is where the actual
// closed-loop control law lives: P reacts to how far off-target the nose
// is, I cleans up whatever P alone can't null out, D would react to the
// vehicle's own spin rate but is tuned to 0 here (see KD below).
//
// The PID doesn't read raw GPS -- it reads a FUSED position estimate
// (see estimatePosition()) blending the noisy GPS fix with the
// accelerometer's own double-integrated motion. Why: GPS's own noise has
// a ~60s correlation time, comparable to this vehicle's whole ~25s
// flight, so within one flight it looks less like jitter and more like a
// near-constant few-meter miscalibration -- no amount of PID retuning
// fixes that. The fused estimate is a real step up from raw GPS, though
// it can't invent an independent ground truth GPS doesn't have either.
//
// Stays neutral below ACTIVATION_ALTITUDE_M -- right off the pad, the
// rocket is slow and barely off vertical, so a tiny angle error would
// swing the PID command toward full gimbal deflection at exactly the
// wrong moment (rail/tower still nearby, no airspeed for TVC to matter
// yet anyway).
class Navigation {
public:
    // Goal: fix the aim point for the whole flight, and size the
    // position-estimator gains once up front from the sim's own dt (see
    // estimatePosition()). Throws if the target is on/near the ground --
    // aiming the nose (and thrust) there is unsurvivable, not just "out
    // of range."
    Navigation(const Eigen::Vector3d& target_position, double dt);

    // Goal: the one call site -- read the current sensors, decide which
    // way to steer, and hand back a body-frame direction for
    // ThrustVectorControl::commandForceDirection. Not const: the
    // integral accumulators and position filter are real memory that
    // persists step to step.
    Eigen::Vector3d computeTvcTarget(const sensors::Gps& gps, const sensors::Gyro& gyro, double dt);

private:
    static constexpr double MIN_TARGET_ALTITUDE_M = 10.0;  // ground-safety floor, meters
    static constexpr double ACTIVATION_ALTITUDE_M = 2.0;   // vehicle altitude before guidance activates

    // ##### PID gains #####
    // Goal: Kp=1, Ki=0, Kd=0 reproduces the old direct-passthrough
    // behavior exactly. Actual values below came from a grid search
    // against this project's own test scenario, not analytic tuning:
    //   - KD hurt at every KI tried (the actuator's own lag already
    //     supplies this loop's damping -- adding more fights it).
    //   - KI helped a lot -- verified again after sensor noise and the
    //     position filter were added, KI=0.6 is still the best performer.
    static constexpr double KP = 1.0;
    static constexpr double KI = 0.6;
    static constexpr double KD = 0.0;

    // Goal: cap how much the integral term alone can contribute, so a
    // long saturated stretch can't build more windup than one actuator
    // swing is worth. Mirrors ThrustVectorControl::MAX_GIMBAL_DEG as its
    // own constant (kept decoupled rather than shared, same pattern
    // sensors.cpp uses for its own ISA formula copy).
    static constexpr double MAX_GIMBAL_RAD = 30.0 * M_PI / 180.0;

    // ##### Position estimator (alpha-beta / g-h filter) #####
    // Goal: track position+velocity, predicting each step from the
    // accelerometer's own measured (noisy) acceleration rather than
    // assuming constant velocity the way a textbook alpha-beta filter
    // does, then correcting both from GPS's residual each step. Gains
    // are derived (Wikipedia's steady-state g-h relation, from this
    // project's own real sensor sigmas), not guessed. Horizontal (x,y)
    // and vertical (z) get separate gains since GPS's own noise differs
    // between them.
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

    // Integral accumulators, one per axis -- real memory between calls.
    double integral_pitch_ = 0.0;
    double integral_yaw_ = 0.0;
};
