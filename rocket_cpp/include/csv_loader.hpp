#pragma once

#include "rocket_types.hpp"
#include <vector>
#include <string>

struct FlightData {
    std::vector<double> time;        // Uniform timestep
    std::vector<double> thrust;      // Thrust at each timestep (N)
    std::vector<double> mass;        // Mass at each timestep (g)
    std::vector<double> drag_coeff;  // Cd at each timestep
    double dry_mass;                 // Mass at burnout (g)
};

// Load flight data from OpenRocket CSV format
FlightData loadFlightData(const std::string& csv_path, double dt);

// Generate thrust profile from known CSV data points (fallback)
std::vector<double> generateThrustProfile(int n_points, double dt);

// Generate constant drag coefficients (fallback)
std::vector<double> generateDragCoeffs(int n_points, double value = 0.867734);

// Generate normal force coefficients (fallback)
std::vector<double> generateNormalCoeffs(int n_points, double value = 0.0);

// Get mass at time using nearest neighbor interpolation
double getMassAtTime(const FlightData& data, double time);