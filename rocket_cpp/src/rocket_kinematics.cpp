#include "rocket_kinematics.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

RocketKinematics::RocketKinematics(const RocketParams& params, const SimulationConfig& config)
    : params_(params), config_(config) {}

Eigen::Matrix3d RocketKinematics::rocketToNedFrame(const RocketState& state) const {
    double roll  = state.orientation(0);
    double pitch = state.orientation(1);
    double yaw   = state.orientation(2);

    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw),   sy = sin(yaw);

    // Rotation from body to NED: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Eigen::Matrix3d R;
    R << cp*cy, cp*sy*sr - sp*cr, cp*sy*cr + sp*sr,
         sy,    cy*sr,           -cy*sr*sp + cr*sy,
         -sp,   cp*sr,           cp*cr;
    return R;
}

Eigen::Vector3d RocketKinematics::computeAcceleration(const RocketState& state,
                                                     double thrust,
                                                     double drag_coeff,
                                                     double normal_coeff) const {
    const double g0 = 9.80665;
    const double mass_kg = state.mass;
    const double v = state.velocity.norm();
    const double v2 = v * v;

    // Thrust along rocket Z axis (body frame)
    Eigen::Vector3d thrust_body(0, 0, thrust);

    // Air density with altitude
    double altitude = state.position(2);
    if (altitude < 0) altitude = 0;
    double rho = 1.225 * exp(-altitude / 8500.0);

    // Drag force: opposes velocity
    const double drag_area = 0.005;  // 50 cm^2
    Eigen::Vector3d drag_neg;
    if (v > 1e-6) {
        drag_neg = -state.velocity.normalized() * drag_coeff * drag_area * 0.5 * rho * v2;
    } else {
        drag_neg = Eigen::Vector3d::Zero();
    }

    // Normal force: perpendicular to velocity and rocket axis
    Eigen::Vector3d up_vec(0, 0, 1);
    Eigen::Vector3d vel_norm = state.velocity.normalized();
    Eigen::Vector3d normal_dir;
    if (vel_norm.cross(up_vec).norm() > 1e-12) {
        normal_dir = vel_norm.cross(up_vec).normalized();
    } else {
        normal_dir = Eigen::Vector3d::UnitX();
    }
    Eigen::Vector3d normal_force = normal_dir * normal_coeff * drag_area * 0.5 * rho * v2;

    // Transform to NED frame
    Eigen::Matrix3d R = rocketToNedFrame(state);
    Eigen::Vector3d thrust_ned = R * thrust_body;
    Eigen::Vector3d drag_ned   = R * drag_neg;
    Eigen::Vector3d normal_ned = R * normal_force;

    // Total force + gravity
    Eigen::Vector3d total_force = thrust_ned + drag_ned + normal_ned;
    Eigen::Vector3d gravity(0, 0, -g0 * mass_kg);

    return (total_force + gravity) / mass_kg;
}

RocketState RocketKinematics::step(const RocketState& state, double thrust,
                                    double drag_coeff, double normal_coeff) const {
    RocketState next = state;

    Eigen::Vector3d accel = computeAcceleration(state, thrust, drag_coeff, normal_coeff);

    // Euler integration
    next.velocity = state.velocity + accel * config_.dt;
    next.position = state.position + next.velocity * config_.dt;

    // Simple pitch rate from lateral acceleration
    double pitch_rate = 0.0;
    if (state.velocity.squaredNorm() > 1e-12) {
        pitch_rate = accel(1) / (state.velocity.norm() + 1e-6) * config_.dt;
    }
    next.orientation(1) = state.orientation(1) + pitch_rate;

    // Keep angles bounded
    next.orientation(0) = fmod(state.orientation(0), 2*M_PI);
    next.orientation(2) = fmod(state.orientation(2), 2*M_PI);

    return next;
}

std::vector<RocketState> RocketKinematics::simulate(double time,
                                                    const std::vector<double>& thrust_data,
                                                    const std::vector<double>& drag_coeffs,
                                                    const std::vector<double>& normal_coeffs) const {
    int n_steps = static_cast<int>(time / config_.dt);
    std::vector<RocketState> states(n_steps + 1);

    // Initial state
    RocketState initial;
    initial.position = Eigen::Vector3d(0, 0, config_.launch_height);
    initial.velocity = Eigen::Vector3d::Zero();
    initial.orientation = Eigen::Vector3d(0,
        config_.init_tilt * M_PI / 180.0,  // pitch from vertical
        0);
    initial.mass = gramsToKg(params_.cg_location);
    states[0] = initial;

    double mass = gramsToKg(params_.cg_location);
    double mass_burn_rate = mass / params_.thrust_duration;

    for (int i = 0; i < n_steps; ++i) {
        double t = i * config_.dt;

        double thrust = 0.0;
        double drag_c  = drag_coeffs.empty() ? 0.0 : drag_coeffs[i % drag_coeffs.size()];
        double normal_c = normal_coeffs.empty() ? 0.0 : normal_coeffs[i % normal_coeffs.size()];

        if (t < params_.thrust_duration) {
            int idx = std::min(static_cast<int>(t / config_.dt), static_cast<int>(thrust_data.size()) - 2);
            double t_frac = (t - idx * config_.dt) / config_.dt;
            thrust = thrust_data[idx] * (1.0 - t_frac) + thrust_data[idx + 1] * t_frac;
            mass = gramsToKg(params_.cg_location) - mass_burn_rate * t;
        }

        RocketState temp = states[i];
        temp.mass = mass;
        states[i + 1] = step(temp, thrust, drag_c, normal_c);

        // Ground termination: interpolate exact impact at z=0
        if (states[i + 1].position(2) < 0.0) {
            double z_above = states[i].position(2);
            double z_below = states[i + 1].position(2);
            if (z_above > 0 && z_below < 0) {
                // Linear interpolation to exact ground contact (z=0)
                double frac = z_above / (z_above - z_below);
                // Interpolate position
                states[i + 1].position = states[i].position + 
                    (states[i + 1].position - states[i].position) * frac;
                // Force exact ground level
                states[i + 1].position(2) = 0.0;
                // Interpolate velocity and orientation
                states[i + 1].velocity = states[i].velocity + 
                    (states[i + 1].velocity - states[i].velocity) * frac;
                states[i + 1].orientation = states[i].orientation + 
                    (states[i + 1].orientation - states[i].orientation) * frac;
                // Force zero velocity and flat orientation at impact
                states[i + 1].velocity.setZero();
                states[i + 1].orientation.setZero();
                // Fill remaining steps at ground level
                for (int j = i + 2; j <= n_steps; ++j) {
                    states[j] = states[i + 1];
                }
                break;
            }
        }
    }
    return states;
}