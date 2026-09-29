#pragma once

#include "sim/rocket_types.hpp"
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <random>

// ##### Simulated flight-computer sensors, the big picture #####
// Goal: each class reads the vehicle's true kinematics (RocketState) and
// adds a real noise model on top -- same "update() then read...()" shape
// real sensor drivers use. One class per physical sensor (not one
// do-everything class), mirroring how real flight computers wire up a
// handful of independent parts rather than a monolith.
namespace sensors {

// ##### GaussMarkovNoise #####
// Goal: model a slowly-varying, bounded random process -- the standard
// way real sensor bias drift and GPS position error behave in practice
// (this is a first-order Gauss-Markov / Ornstein-Uhlenbeck process,
// discretized exactly). Each sample() call decays the current value a
// little toward zero, then adds a fresh random kick -- so its OWN
// standard deviation settles at steady_state_sigma instead of growing
// without bound the way a plain random walk would. Reused below for
// gyro/accelerometer bias, GPS position drift, and barometer
// environmental drift.
class GaussMarkovNoise {
public:
    GaussMarkovNoise(double steady_state_sigma, double correlation_time_s)
        : sigma_(steady_state_sigma), tau_s_(correlation_time_s) {}

    // Goal: exact discretization of the underlying stochastic equation --
    // value_'s own standard deviation stays sigma_ regardless of dt, not
    // an artifact of the step size chosen.
    double sample(double dt, std::mt19937& rng) {
        std::normal_distribution<double> white(0.0, 1.0);
        double a = std::exp(-dt / tau_s_);
        value_ = value_ * a + sigma_ * std::sqrt(1.0 - a * a) * white(rng);
        return value_;
    }

private:
    double sigma_;
    double tau_s_;
    double value_ = 0.0;
};

// ##### Gyro (the IMU) #####
// Goal: accelerometer + gyroscope in one chip, mirroring the actual
// MPU-9250 this project's flight computer uses (see artifacts/
// PS-MPU-9250A-01-v1.1-1313803.pdf). Noise numbers below come straight
// from that datasheet where it gives them; general consumer-MEMS
// literature fills in bias instability, which the datasheet doesn't
// quote directly. Two noise terms per channel: white noise (the
// datasheet's own RMS spec) plus a slowly wandering bias
// (GaussMarkovNoise).
class Gyro {
public:
    // Goal: compute this step's readings from the true kinematics.
    // state/prev_state + dt give the accelerometer and gyroscope their
    // readings (both are rate-of-change quantities -- need two samples,
    // not one).
    void update(const RocketState& state, const RocketState& prev_state, double dt);

    // Goal: report proper (specific) acceleration, body frame -- what an
    // accelerometer's proof mass actually feels. NOT the same as total
    // kinematic acceleration: a free-falling vehicle reads ~0 on every
    // axis, since gravity pulls the proof mass and the case around it
    // equally, leaving nothing to sense.
    Eigen::Vector3d readAccel() const { return accel_body_; }

    // Goal: report angular acceleration, differentiated from this
    // sensor's OWN (already noisy) rate output, not the true state --
    // reproducing how noise gets amplified when a real system
    // numerically differentiates real gyro data.
    Eigen::Vector3d readAngularAccel() const { return angular_accel_; }

    // Goal: report angular rate (p,q,r) directly -- what a gyroscope
    // actually measures, no differentiation needed.
    Eigen::Vector3d readAngularVel() const { return angular_vel_; }

    // Goal: report true orientation, deliberately noise-free -- a real
    // IMU doesn't hand you absolute attitude for free (that needs
    // integrating/fusing the rates above, or a magnetometer), so this is
    // a direct pass-through rather than pretending to model an estimator
    // that isn't built.
    Eigen::Vector3d readOrientation() const { return orientation_; }

private:
    static constexpr double GYRO_WHITE_NOISE_STD_RAD_S = 0.1 * M_PI / 180.0;
    // ^ datasheet Table 1: "Total RMS Noise" = 0.1 deg/s-rms @ 92Hz (DLPFCFG=2).

    static constexpr double GYRO_BIAS_SIGMA_RAD_S = (20.0 / 3600.0) * M_PI / 180.0;
    // ^ ~20 deg/h bias instability -- not in the datasheet directly;
    // representative mid-range figure for consumer-grade MEMS gyros.

    static constexpr double ACCEL_WHITE_NOISE_STD_MPS2 = 0.008 * 9.80665;
    // ^ datasheet Table 2: "Total RMS Noise" = 8 mg-rms @ 94Hz (DLPFCFG=2).

    static constexpr double ACCEL_BIAS_SIGMA_MPS2 = 0.002 * 9.80665;
    // ^ ~2 mg -- order-of-magnitude estimate from the datasheet's own
    // temperature coefficient over a modest in-flight temperature swing.

    static constexpr double BIAS_TAU_S = 100.0;
    // ^ Long relative to a ~25s flight, so bias looks like a
    // near-constant small offset within one flight -- real bias
    // instability only wanders meaningfully over minutes/hours.

    Eigen::Vector3d accel_body_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_accel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d orientation_ = Eigen::Vector3d::Zero();

    std::mt19937 rng_{std::random_device{}()};
    std::array<GaussMarkovNoise, 3> gyro_bias_{
        GaussMarkovNoise(GYRO_BIAS_SIGMA_RAD_S, BIAS_TAU_S),
        GaussMarkovNoise(GYRO_BIAS_SIGMA_RAD_S, BIAS_TAU_S),
        GaussMarkovNoise(GYRO_BIAS_SIGMA_RAD_S, BIAS_TAU_S)};
    std::array<GaussMarkovNoise, 3> accel_bias_{
        GaussMarkovNoise(ACCEL_BIAS_SIGMA_MPS2, BIAS_TAU_S),
        GaussMarkovNoise(ACCEL_BIAS_SIGMA_MPS2, BIAS_TAU_S),
        GaussMarkovNoise(ACCEL_BIAS_SIGMA_MPS2, BIAS_TAU_S)};
};

// ##### Baro (barometric altimeter) #####
// Goal: measure pressure, report the altitude a real baro derives from
// it via the standard ISA formula -- genuinely inverted (see .cpp), not
// shortcut, since real noise on the pressure reading gives that
// inversion something to actually do. No specific chip is picked for
// this project yet, so the numbers here are the Bosch BMP388 (a common
// hobby-rocketry choice) plus the general finding that barometric noise
// splits into a dominant electronic term and a smaller environmental-
// drift term.
class Baro {
public:
    void update(const RocketState& state, double dt);
    double readPressure() const { return pressure_pa_; }
    double readAltitude() const { return altitude_m_; }

private:
    static constexpr double PRESSURE_WHITE_NOISE_STD_PA = 0.016;  // BMP388 datasheet RMS noise
    static constexpr double PRESSURE_DRIFT_SIGMA_PA = 2.0;        // environmental drift, order-of-magnitude estimate
    static constexpr double DRIFT_TAU_S = 30.0;

    double pressure_pa_ = 0.0;
    double altitude_m_ = 0.0;
    std::mt19937 rng_{std::random_device{}()};
    GaussMarkovNoise pressure_drift_{PRESSURE_DRIFT_SIGMA_PA, DRIFT_TAU_S};
};

// ##### Gps #####
// Goal: report a world-frame position fix. No specific GPS module is
// picked for this project yet, so the numbers here are representative
// consumer/hobby GNSS figures. Position error is modeled per axis as a
// first-order Gauss-Markov process (not independent sample-to-sample
// noise), since real GPS error sources (multipath, atmospheric delay)
// drift over tens of seconds rather than resetting every sample.
// Vertical accuracy is worse than horizontal by a well-established
// ~1.7x ratio.
class Gps {
public:
    void update(const RocketState& state, double dt);
    Eigen::Vector3d readPosition() const { return position_; }

private:
    static constexpr double HORIZONTAL_SIGMA_M = 2.5;         // representative consumer-GNSS 1-sigma horizontal
    static constexpr double VERTICAL_SIGMA_M = 2.5 * 1.7;      // ~1.7x horizontal, standard GNSS ratio
    static constexpr double POSITION_TAU_S = 60.0;             // typical GPS error correlation time

    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    std::mt19937 rng_{std::random_device{}()};
    GaussMarkovNoise north_{HORIZONTAL_SIGMA_M, POSITION_TAU_S};  // world +X
    GaussMarkovNoise east_{HORIZONTAL_SIGMA_M, POSITION_TAU_S};   // world +Y
    GaussMarkovNoise up_{VERTICAL_SIGMA_M, POSITION_TAU_S};       // world +Z
};

}  // namespace sensors
