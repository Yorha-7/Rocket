#pragma once

#include "sim/rocket_types.hpp"
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <random>

// Simulated flight-computer sensors: each one reads the vehicle's true
// kinematics (RocketState) and adds a real noise model on top -- the same
// "update() then read...()" shape real sensor drivers use. One namespace,
// one class per physical sensor (not one do-everything class) -- mirrors
// how real flight computers wire up a handful of independent parts, not
// a monolith.
namespace sensors {

// A slowly-varying, physically-bounded random process -- a first-order
// Gauss-Markov process (an Ornstein-Uhlenbeck process, discretized
// exactly), the standard way real sensor bias drift and GPS position
// error are modeled in industry practice (MATLAB's own gpsSensor block
// models position noise exactly this way). value_ decays toward zero
// with time constant tau_s while getting kicked by fresh white noise
// every step, so its OWN standard deviation converges to
// steady_state_sigma rather than growing without bound the way a plain
// random walk would -- appropriate here since a real flight lasts tens
// of seconds, far shorter than the timescale bias instability needs to
// actually wander unboundedly.
//
// Reused below for gyro/accelerometer bias drift, GPS position drift,
// and barometer environmental drift -- one mechanism standing in for
// what a full Allan-variance analysis would split into several distinct
// noise terms (angle/rate random walk, bias instability, ...) per
// sensor. More rigor than a staging-area sim like this one needs; see
// each class's own doc comment for the actual numbers used and where
// they came from.
class GaussMarkovNoise {
public:
    GaussMarkovNoise(double steady_state_sigma, double correlation_time_s)
        : sigma_(steady_state_sigma), tau_s_(correlation_time_s) {}

    double sample(double dt, std::mt19937& rng) {
        std::normal_distribution<double> white(0.0, 1.0);
        double a = std::exp(-dt / tau_s_);
        // Exact discretization of the OU SDE: value_'s own standard
        // deviation is sigma_ regardless of dt, not an artifact of step size.
        value_ = value_ * a + sigma_ * std::sqrt(1.0 - a * a) * white(rng);
        return value_;
    }

private:
    double sigma_;
    double tau_s_;
    double value_ = 0.0;
};

// The IMU: accelerometer + gyroscope (a magnetometer is common on real
// IMU chips too, but left out for now -- nothing needs heading yet).
// Noise numbers below are the actual MPU-9250 (this project's own
// flight-computer IMU -- see artifacts/PS-MPU-9250A-01-v1.1-1313803.pdf)
// datasheet specs where it gives them directly; general consumer-MEMS
// literature fills in bias instability, which that datasheet doesn't
// quote. Two terms per channel: white noise (the datasheet's own RMS
// spec) plus a slowly wandering bias (GaussMarkovNoise).
class Gyro {
public:
    // Computes this step's reading from the true kinematics. state/
    // prev_state + dt give the accelerometer and gyroscope their readings
    // (both are rate-of-change sensors -- they need two samples, not one).
    void update(const RocketState& state, const RocketState& prev_state, double dt);

    // Proper (specific) acceleration, body frame, m/s^2 -- what an
    // accelerometer actually measures. NOT the same as total kinematic
    // acceleration: an accelerometer's proof mass doesn't feel gravity
    // (a free-falling vehicle reads ~0 on every axis), so this is total
    // acceleration with gravity subtracted back out before rotating into
    // body frame.
    Eigen::Vector3d readAccel() const { return accel_body_; }

    // Angular acceleration, body-ish frame (finite difference of
    // angular_vel between the two states), rad/s^2. Differentiated from
    // this sensor's OWN (already noisy) rate output, not the true state
    // -- exactly what a real system computing this from raw gyro data
    // would have to do, noise and all.
    Eigen::Vector3d readAngularAccel() const { return angular_accel_; }

    // Angular rate (p, q, r), body frame, rad/s -- what a gyroscope
    // actually measures directly (unlike accel/angular-accel above, no
    // finite difference needed: rate is already a first-derivative
    // quantity RocketState carries natively).
    Eigen::Vector3d readAngularVel() const { return angular_vel_; }

    // True orientation (roll, pitch, yaw), rad -- a real IMU doesn't hand
    // you absolute attitude for free, that takes integrating/fusing the
    // rates above (or a magnetometer/star tracker). That estimator isn't
    // built yet, so this is a direct pass-through of the true value --
    // deliberately left noise-free (adding noise to a value nothing
    // actually measures wouldn't model anything real; see readAngularVel
    // above for the sensor reading that's actually noisy).
    Eigen::Vector3d readOrientation() const { return orientation_; }

private:
    static constexpr double GYRO_WHITE_NOISE_STD_RAD_S = 0.1 * M_PI / 180.0;
    // ^ datasheet Table 1: "Total RMS Noise" = 0.1 deg/s-rms @ 92Hz (DLPFCFG=2).

    static constexpr double GYRO_BIAS_SIGMA_RAD_S = (20.0 / 3600.0) * M_PI / 180.0;
    // ^ ~20 deg/h bias instability -- the datasheet doesn't quote this
    // directly; 20 deg/h is a representative mid-range figure for
    // consumer-grade MEMS gyros generally (tactical-grade parts do much
    // better, cheaper parts worse).

    static constexpr double ACCEL_WHITE_NOISE_STD_MPS2 = 0.008 * 9.80665;
    // ^ datasheet Table 2: "Total RMS Noise" = 8 mg-rms @ 94Hz (DLPFCFG=2).

    static constexpr double ACCEL_BIAS_SIGMA_MPS2 = 0.002 * 9.80665;
    // ^ ~2 mg -- order-of-magnitude estimate from the datasheet's own
    // "Zero-G Level Change vs Temperature" (+-1.5 mg/degC) over a modest
    // few-degree in-flight temperature swing, not a directly-quoted spec.

    static constexpr double BIAS_TAU_S = 100.0;
    // ^ Correlation time long relative to a ~25s flight -- within one
    // flight the bias looks like a near-constant small offset, exactly
    // how real bias instability behaves at these timescales (it only
    // wanders meaningfully over minutes/hours, not tens of seconds).

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

// Barometric altimeter: measures pressure, reports the altitude a real
// baro derives from it via the standard ISA formula -- now genuinely
// inverted (see .cpp), since with real noise on the pressure reading
// there's something for that inversion to actually do. No specific
// barometer chip is chosen for this project yet, so the numbers below
// are the Bosch BMP388 (a common choice for hobby-rocketry altimeters)
// plus the general finding (Stochastic Approach to Noise Modeling for
// Barometric Altimeters, Sensors 2013) that barometric noise splits into
// a dominant uncorrelated (electronic/quantization) term and a smaller
// correlated term from short-term environmental change.
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

// GPS: world-frame position fix. Real GPS also reports velocity, but
// nothing needs that yet. No specific GPS module is chosen for this
// project yet, so the numbers below are representative consumer/hobby
// GNSS figures. Position error is modeled per axis as a first-order
// Gauss-Markov process, not independent sample-to-sample white noise --
// the standard industry approach (MATLAB's gpsSensor does the same),
// since real GPS error sources (multipath, atmospheric delay, ephemeris)
// drift over tens of seconds rather than resetting every sample.
// Vertical accuracy is worse than horizontal by a well-established
// ~1.7x ratio (GNSS accuracy literature).
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
