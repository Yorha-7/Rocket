#include "gnc/sensors.hpp"
#include "sim/rocket_kinematics.hpp"
#include <cmath>

namespace sensors {

// ##### Gyro::update() #####
// Goal: work out this step's noisy accelerometer + gyroscope readings
// from the vehicle's true state.
void Gyro::update(const RocketState& state, const RocketState& prev_state, double dt) {
    const double g0 = 9.80665;

    // Goal: get total (kinematic) world-frame acceleration from the
    // change in velocity, then subtract gravity to get what an
    // accelerometer's proof mass actually feels -- a free-falling
    // vehicle reads ~0, not -g, on every axis, since gravity pulls the
    // proof mass exactly like everything around it, leaving no relative
    // force to sense.
    Eigen::Vector3d total_accel_world = Eigen::Vector3d::Zero();
    if (dt > 1e-9) {
        total_accel_world = (state.velocity - prev_state.velocity) / dt;
    }
    Eigen::Vector3d gravity_world(0, 0, -g0);
    Eigen::Vector3d proper_accel_world = total_accel_world - gravity_world;

    // Goal: rotate that into BODY frame -- an accelerometer measures
    // along its own case's axes, not world axes. Same DCM the integrator
    // itself uses, transposed (world->body is the reverse of body->world).
    Eigen::Matrix3d R = RocketKinematics::rocketToNedFrame(state);
    Eigen::Vector3d true_accel_body = R.transpose() * proper_accel_world;
    Eigen::Vector3d true_angular_vel = state.angular_vel;

    // Goal: remember last step's already-noisy rate reading, so
    // angular_accel_ below differentiates THAT (not the true state) --
    // genuinely reproducing how a real system's noise gets amplified
    // when it numerically differentiates a real gyro's output.
    Eigen::Vector3d angular_vel_prev_noisy = angular_vel_;

    // Goal: add this sensor's actual noise -- white noise (datasheet RMS)
    // plus a slowly wandering bias -- to both channels, one axis at a time.
    std::normal_distribution<double> accel_white(0.0, ACCEL_WHITE_NOISE_STD_MPS2);
    std::normal_distribution<double> gyro_white(0.0, GYRO_WHITE_NOISE_STD_RAD_S);
    for (int i = 0; i < 3; ++i) {
        accel_body_(i) = true_accel_body(i) + accel_white(rng_) + accel_bias_[i].sample(dt, rng_);
        angular_vel_(i) = true_angular_vel(i) + gyro_white(rng_) + gyro_bias_[i].sample(dt, rng_);
    }

    angular_accel_ = Eigen::Vector3d::Zero();
    if (dt > 1e-9) {
        angular_accel_ = (angular_vel_ - angular_vel_prev_noisy) / dt;
    }

    orientation_ = state.orientation;
}

// ##### Baro::update() #####
// Goal: compute true pressure at this altitude (same troposphere ISA
// formula AerodynamicsModel::getDensity() uses -- duplicated rather than
// shared, since Sensors is deliberately decoupled from the aero/
// kinematics internals, the way a real flight computer's baro driver
// wouldn't reach into the vehicle's own drag model either), add this
// sensor's noise, then invert the SAME formula to turn that noisy
// pressure back into an altitude reading -- a real barometric
// altimeter's actual job.
void Baro::update(const RocketState& state, double dt) {
    const double T0 = 288.15, P0 = 101325.0, LAPSE_RATE = -0.0065;
    const double G0 = 9.80665, R_GAS = 287.058;

    double altitude = std::max(0.0, state.position(2));
    double T = T0 + LAPSE_RATE * altitude;
    double exponent = -G0 / (LAPSE_RATE * R_GAS);
    double true_pressure = P0 * std::pow(T / T0, exponent);

    std::normal_distribution<double> white(0.0, PRESSURE_WHITE_NOISE_STD_PA);
    pressure_pa_ = true_pressure + white(rng_) + pressure_drift_.sample(dt, rng_);

    // Goal: invert pressure -> altitude. Genuinely differs from the true
    // altitude now that there's real noise to invert (it used to
    // short-circuit straight to the true value when noise didn't exist).
    double ratio = pressure_pa_ / P0;
    if (ratio > 0.0) {
        double T_est = T0 * std::pow(ratio, 1.0 / exponent);
        altitude_m_ = (T_est - T0) / LAPSE_RATE;
    } else {
        altitude_m_ = altitude;  // pathological (near-vacuum) noisy reading -- fall back rather than NaN
    }
}

// ##### Gps::update() #####
// Goal: report the true position plus this sensor's own (correlated,
// slowly-drifting) position noise, one Gauss-Markov process per axis.
void Gps::update(const RocketState& state, double dt) {
    Eigen::Vector3d noise(north_.sample(dt, rng_), east_.sample(dt, rng_), up_.sample(dt, rng_));
    position_ = state.position + noise;
}

}  // namespace sensors
