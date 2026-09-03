#include "rocket_kinematics.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <iostream>

RocketKinematics::RocketKinematics(const RocketParams& params, const SimulationConfig& config)
    : params_(params), config_(config) {}

Eigen::Matrix3d RocketKinematics::rocketToNedFrame(const RocketState& state) const {
    double roll  = state.orientation(0);
    double pitch = state.orientation(1);
    double yaw   = state.orientation(2);

    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw),   sy = sin(yaw);

    Eigen::Matrix3d R;
    R << cp*cy, cp*sy*sr - sp*cr, cp*sy*cr + sp*sr,
         sy,    cy*sr,           -cy*sr*sp + cr*sy,
         -sp,   cp*sr,           cp*cr;
    return R;
}

Eigen::Vector3d RocketKinematics::computeAcceleration(const RocketState& state,
                                                     double thrust,
                                                     double drag_coeff) const {
    const double g0 = 9.80665;
    const double mass_kg = state.mass;
    const double v = state.velocity.norm();
    const double v2 = v * v;

    Eigen::Vector3d thrust_body(0, 0, thrust);

    double altitude = state.position(2);
    if (altitude < 0) altitude = 0;
    double rho = 1.225 * exp(-altitude / 8500.0);

    const double drag_area = 0.005;

    Eigen::Vector3d drag_neg;
    if (v > 1e-6) {
        drag_neg = -state.velocity.normalized() * drag_coeff * drag_area * 0.5 * rho * v2;
    } else {
        drag_neg = Eigen::Vector3d::Zero();
    }

    Eigen::Matrix3d R = rocketToNedFrame(state);
    Eigen::Vector3d thrust_ned = R * thrust_body;
    Eigen::Vector3d drag_ned   = R * drag_neg;

    Eigen::Vector3d total_force = thrust_ned + drag_ned;
    Eigen::Vector3d gravity(0, 0, -g0 * mass_kg);

    return (total_force + gravity) / mass_kg;
}

// ============================================================
// Aerodynamic Coefficient Calculations
// ============================================================

// Calculate Cn_alpha (normal force coefficient derivative) from rocket geometry
// Based on slender body theory + empirical corrections
// Cn_alpha = Cn_alpha_body + Cn_alpha_fins
// For body: Cn_alpha_body = 2 * (1 - d/L) * (1 - x_cg/L) + base_correction
// For fins: Cn_alpha_fins = N_fins * AR * (span/d)^2 * CL_alpha_fin / (pi/4)
double RocketKinematics::calculateCnAlpha() const {
    // Reference area A = 0.005 m^2 -> diameter d
    const double A = 0.005;
    double d = sqrt(4.0 * A / M_PI);  // diameter in meters
    
    // CP and CG locations from tip (in cm)
    double cp_cm = params_.cp_location;
    double cg_cm = params_.cg_location;
    
    // Estimate rocket length from CP location
    // For stable rockets, CP is typically at 60-80% of body length from nose
    // Using 75% as average
    double L_cm = cp_cm / 0.75;
    double L_m = L_cm / 100.0;  // meters
    
    // CG position from nose (m)
    double x_cg = cg_cm / 100.0;
    
    // Body diameter
    double d_m = d;
    
    // Slender body theory for body Cn_alpha
    // Cn_alpha_body = 2 * (1 - d/L) * (1 - x_cg/L)
    // Plus base correction: base = d/L
    double L = L_m;
    double d_over_L = d_m / L;
    double x_cg_over_L = x_cg / L;
    
    double cn_alpha_body = 2.0 * (1.0 - d_over_L) * (1.0 - x_cg_over_L) + d_over_L;
    
    // Fin contribution (simplified - no fin data available)
    // For finned rockets: Cn_alpha_fins = N * AR * (span/d)^2 * CL_alpha_fin / (pi/4)
    // CL_alpha_fin ~ 2*pi for thin flat plates
    // AR = span/chord
    // Without fin data, use conservative factor
    double cn_alpha_fins = 0.0;  // No fin data available
    
    // Total Cn_alpha (per radian)
    double cn_alpha = cn_alpha_body + cn_alpha_fins;
    
    // Clamp to reasonable range
    double result = std::max(0.1, std::min(5.0, cn_alpha));
    static bool printed = false;
    if (!printed) {
        std::cout << "Calculated Cn_alpha: " << result << " per radian\n";
        printed = true;
    }
    return result;
}

// ============================================================
// Pitch Dynamics - Modular Methods
// ============================================================

// 1. Gravity torque from CG-CP offset
// CP behind CG => positive moment arm (d = CP - CG > 0)
// Gravity pulls down on CG, creating restoring torque for positive pitch (nose up)
// Torque = -m * g * d * sin(pitch)  (negative for positive pitch = restoring)
double RocketKinematics::computeGravityTorque(const RocketState& state) const {
    double cp_cg_diff_m = (params_.cp_location - params_.cg_location) / 100.0; // m
    double mass_kg = state.mass;
    double pitch = state.orientation(1); // pitch in radians
    
    // Torque = -m * g * d * sin(pitch)
    // Negative for positive pitch => restoring torque
    return -mass_kg * 9.80665 * (params_.cp_location - params_.cg_location) / 100.0 * sin(state.orientation(1));
}

// 2. Aerodynamic moment - using calculated Cn_alpha
// Moment = -0.5 * rho * v^2 * Cn_alpha * alpha * A * d
// Negative sign: positive alpha => negative moment (restoring)
double RocketKinematics::computeAerodynamicMoment(const RocketState& state) const {
    double v = state.velocity.norm();
    if (v < 1e-6) return 0.0;
    
    double altitude = state.position(2);
    if (altitude < 0) altitude = 0;
    double rho = 1.225 * exp(-altitude / 8500.0);
    
    double d = (params_.cp_location - params_.cg_location) / 100.0; // m
    const double A = 0.005; // reference area m^2
    
    // Calculate Cn_alpha from rocket geometry
    double Cn_alpha = calculateCnAlpha();
    
    // Angle of attack approximation: pitch angle (for near-vertical flight)
    double alpha = state.orientation(1);
    
    // Limit alpha to prevent instability
    double alpha_limited = std::max(-0.5, std::min(0.5, alpha));
    
    // Aerodynamic moment = -0.5 * rho * v^2 * Cn_alpha * alpha * A * d
    // Negative sign: positive alpha => negative moment (restoring)
    return -0.5 * rho * state.velocity.squaredNorm() * Cn_alpha * alpha_limited * 0.005 * d;
}

// 3. Damping torque (empirical, velocity-dependent)
// Uses OpenRocket damping factor of 0.6
double RocketKinematics::computeDampingTorque(const RocketState& state) const {
    double v = state.velocity.norm();
    double altitude = state.position(2);
    if (altitude < 0) altitude = 0;
    double rho = 1.225 * exp(-altitude / 8500.0);
    
    double d = (params_.cp_location - params_.cg_location) / 100.0; // m
    const double A = 0.005;
    
    // Damping torque coefficient: C_damp = 0.6 * 0.5 * rho * v * d^2 * A
    // Where d = CP - CG distance
    double c_damp = 0.6 * 0.5 * rho * state.velocity.norm() * 
                    (params_.cp_location - params_.cg_location) * (params_.cp_location - params_.cg_location) / 10000.0 * 0.005;
    
    // Damping torque opposes angular velocity (pitch axis = angular_vel(1))
    return -c_damp * state.angular_vel(1);
}

// Total pitch torque = gravity + aerodynamic + damping
double RocketKinematics::computeTotalPitchTorque(const RocketState& state) const {
    return computeGravityTorque(state) 
         + computeAerodynamicMoment(state)
         + computeDampingTorque(state);
}

// Angular acceleration = total torque / moment of inertia (I_yy for pitch)
double RocketKinematics::computePitchAcceleration(double total_torque) const {
    double alpha = total_torque / params_.I_yy;
    // Limit angular acceleration to prevent instability
    const double MAX_ALPHA = 100.0; // rad/s^2
    return std::max(-MAX_ALPHA, std::min(MAX_ALPHA, alpha));
}

// Update pitch dynamics: integrate angular acceleration -> angular velocity -> pitch
void RocketKinematics::updatePitchDynamics(RocketState& next, const RocketState& state) const {
    double total_torque = computeTotalPitchTorque(state);
    double alpha = computePitchAcceleration(total_torque);
    
    // Integrate angular acceleration to get new angular velocity
    next.angular_vel(1) = state.angular_vel(1) + alpha * config_.dt;
    
    // Limit angular velocity to prevent instability
    const double MAX_ANGULAR_VEL = 10.0; // rad/s
    next.angular_vel(1) = std::max(-MAX_ANGULAR_VEL, std::min(MAX_ANGULAR_VEL, next.angular_vel(1)));
    
    // Integrate angular velocity to get new pitch angle
    next.orientation(1) = state.orientation(1) + next.angular_vel(1) * config_.dt;
    
    // Limit pitch angle to prevent instability
    const double MAX_PITCH = 1.5; // ~85 degrees
    next.orientation(1) = std::max(-MAX_PITCH, std::min(MAX_PITCH, next.orientation(1)));
}

RocketState RocketKinematics::step(const RocketState& state, double thrust,
                                    double drag_coeff) const {
    RocketState next = state;

    Eigen::Vector3d accel = computeAcceleration(state, thrust, drag_coeff);

    next.velocity = state.velocity + accel * config_.dt;
    next.position = state.position + next.velocity * config_.dt;

    // Update pitch dynamics (modular)
    updatePitchDynamics(next, state);

    // Keep roll and yaw bounded
    next.orientation(0) = fmod(state.orientation(0), 2*M_PI);
    next.orientation(2) = fmod(state.orientation(2), 2*M_PI);

    return next;
}

std::vector<RocketState> RocketKinematics::simulate(double time,
                                                    const FlightData& flight_data) const {
    int n_steps = static_cast<int>(time / config_.dt);
    if (n_steps >= (int)flight_data.time.size()) {
        n_steps = flight_data.time.size() - 1;
    }
    std::vector<RocketState> states(n_steps + 1);

    RocketState initial;
    initial.position = Eigen::Vector3d(0, 0, config_.launch_height);
    initial.velocity = Eigen::Vector3d::Zero();
    initial.orientation = Eigen::Vector3d(0,
        config_.init_tilt * M_PI / 180.0,
        0);
    initial.angular_vel = Eigen::Vector3d::Zero(); // Start with zero angular velocity
    initial.mass = gramsToKg(flight_data.mass.front());
    states[0] = initial;

    for (int i = 0; i < n_steps; ++i) {
        double thrust = flight_data.thrust[i];
        double drag_c = flight_data.drag_coeff[i];
        double mass_kg = gramsToKg(flight_data.mass[i]);

        RocketState temp = states[i];
        temp.mass = mass_kg;
        states[i + 1] = step(temp, thrust, drag_c);

        if (states[i + 1].position(2) < 0.0) {
            double z_above = states[i].position(2);
            double z_below = states[i + 1].position(2);
            if (z_above > 0 && z_below < 0) {
                double frac = z_above / (z_above - z_below);
                states[i + 1].position = states[i].position + 
                    (states[i + 1].position - states[i].position) * frac;
                states[i + 1].position(2) = 0.0;
                states[i + 1].velocity = states[i].velocity + 
                    (states[i + 1].velocity - states[i].velocity) * frac;
                states[i + 1].orientation = states[i].orientation + 
                    (states[i + 1].orientation - states[i].orientation) * frac;
                states[i + 1].angular_vel = states[i].angular_vel + 
                    (states[i + 1].angular_vel - states[i].angular_vel) * frac;
                states[i + 1].velocity.setZero();
                states[i + 1].orientation.setZero();
                states[i + 1].angular_vel.setZero();
                states.resize(i + 2);
                return states;
            }
        }
    }
    return states;
}
