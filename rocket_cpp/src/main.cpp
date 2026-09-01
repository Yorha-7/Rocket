#include "rocket_kinematics.hpp"
#include "csv_loader.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <cstdlib>
#include <unistd.h>

// Helper: normalize vector to [0, 1] range
static std::vector<double> normalize(const std::vector<double>& v) {
    if (v.empty()) return {};
    double vmin = *std::min_element(v.begin(), v.end());
    double vmax = *std::max_element(v.begin(), v.end());
    if (vmax == vmin) return std::vector<double>(v.size(), 0.5);
    std::vector<double> out(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        out[i] = (v[i] - vmin) / (vmax - vmin);
    }
    return out;
}

int main() {
    // ============================================================
    // Configuration
    // ============================================================
    RocketParams params;
    params.cp_location = 20.0;
    params.cg_location = 64.528053;
    params.I_xx = 7.972923e-4;
    params.I_yy = 9.682337e-6;
    params.I_zz = 9.682337e-6;
    params.thrust_duration = 1.86;
    params.max_thrust = 4.448;

    SimulationConfig config;
    config.launch_height = 0.0;
    config.init_tilt = 2.0;
    config.sim_duration = 93.511;
    config.dt = 0.01;

    // ============================================================
    // Load flight data (thrust, drag, normal coefficients)
    // ============================================================
    int n_points = static_cast<int>(config.sim_duration / config.dt) + 1;
    auto thrust_data   = generateThrustProfile(n_points, config.dt);
    auto drag_coeffs   = generateDragCoeffs(n_points, 0.867734);
    auto normal_coeffs = generateNormalCoeffs(n_points, 0.0);

    // ============================================================
    // Run simulation
    // ============================================================
    RocketKinematics sim(params, config);

    std::cout << "Rocket Simulation (3DOF) with Ground Termination\n";
    std::cout << "==================================================\n";
    std::cout << "Config: launch_height=" << config.launch_height
              << "m, init_tilt=" << config.init_tilt
              << "deg, dt=" << config.dt << "s\n";

    auto states = sim.simulate(config.sim_duration, thrust_data, drag_coeffs, normal_coeffs);

    // ============================================================
    // Extract time series for output and plotting
    // ============================================================
    std::vector<double> time_vec, height_vec, velocity_vec, pitch_vec;
    time_vec.reserve(states.size());
    height_vec.reserve(states.size());
    velocity_vec.reserve(states.size());
    pitch_vec.reserve(states.size());

    for (size_t i = 0; i < states.size(); ++i) {
        time_vec.push_back(i * config.dt);
        height_vec.push_back(states[i].position(2));
        velocity_vec.push_back(states[i].velocity.norm());
        pitch_vec.push_back(states[i].orientation(1) * 180.0 / M_PI);
    }

    // Compute acceleration from velocity derivative
    std::vector<double> accel_vec(states.size(), 0.0);
    for (size_t i = 1; i < states.size(); ++i) {
        accel_vec[i] = (velocity_vec[i] - velocity_vec[i-1]) / config.dt;
    }
    accel_vec[0] = accel_vec[1];

    // Find key events
    const double t_burnout = 1.86;
    double t_apogee = 0.0, h_apogee = 0.0;
    for (size_t i = 0; i < states.size(); ++i) {
        if (states[i].position(2) > h_apogee) {
            h_apogee = states[i].position(2);
            t_apogee = i * config.dt;
        }
    }
    double t_impact = time_vec.back();

    // ============================================================
    // Save CSV output
    // ============================================================
    std::ofstream csv("rocket_trajectory.csv");
    csv << "time,height,velocity,pitch\n";
    for (size_t i = 0; i < states.size(); ++i) {
        csv << std::fixed << std::setprecision(6);
        csv << time_vec[i] << "," << height_vec[i] << "," << velocity_vec[i] << "," << pitch_vec[i] << "\n";
    }
    csv.close();

    // ============================================================
    // Generate PNG plot via Python/matplotlib
    // ============================================================
    std::cout << "\nGenerating plot via Python/matplotlib...\n";
    // Change to project root so script finds CSV and writes PNG there
    const char* project_root = "/media/jayesh/Acer/Users/scien/Rocket/rocket_cpp";
    if (chdir(project_root) == 0) {
        int result = system("python3 scripts/plot_trajectory.py rocket_trajectory.csv rocket_trajectory.png");
        if (result != 0) {
            std::cerr << "Warning: Python plot generation failed (matplotlib not installed?). Continuing...\n";
        } else {
            std::cout << "Plot saved to rocket_trajectory.png\n";
        }
    } else {
        std::cerr << "Warning: Could not change to project root directory. Skipping plot.\n";
    }

    // ============================================================
    // Console summary
    // ============================================================
    std::cout << "\nTime(s)\tHeight(m)\tVelocity(m/s)\tPitch(deg)\n";
    for (size_t i = 0; i < states.size(); ++i) {
        std::cout << std::fixed << std::setprecision(4)
                  << time_vec[i] << "\t" << height_vec[i] << "\t"
                  << velocity_vec[i] << "\t" << pitch_vec[i] << "\n";
    }

    std::cout << "\nSimulation complete.\n";
    std::cout << "Results: rocket_trajectory.csv, rocket_trajectory.png\n";
    std::cout << "Ground termination: stops when z < 0\n";

    return 0;
}