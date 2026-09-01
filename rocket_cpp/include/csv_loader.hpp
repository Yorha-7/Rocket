#pragma once

#include "rocket_types.hpp"
#include <vector>

struct FlightData {
    std::vector<double> time;
    std::vector<double> thrust;
    std::vector<double> drag_coeff;
    std::vector<double> normal_coeff;
};

// Load flight data from OpenRocket CSV format
FlightData loadFlightData(const std::string& csv_path, double dt);

// Generate thrust profile from known CSV data points
std::vector<double> generateThrustProfile(int n_points, double dt);

// Generate constant drag coefficients
std::vector<double> generateDragCoeffs(int n_points, double value = 0.867734);

// Generate normal force coefficients (zero for now)
std::vector<double> generateNormalCoeffs(int n_points, double value = 0.0);